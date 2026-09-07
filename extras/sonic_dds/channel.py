"""DDS channel init — same as gear_sonic simulator_factory.init_channel.

Stock unitree_sdk2py has no peer_address. That kwarg is a GEAR Tailscale patch
in GR00T's vendored tree. Local sim2sim on ``lo`` uses the stock API.
"""

from __future__ import annotations

from typing import Any

from unitree_sdk2py.core.channel import ChannelFactoryInitialize


def init_channel(config: dict[str, Any]) -> None:
    """Initialize CycloneDDS for sim2sim (one domain per process)."""
    peer_address = config.get("DDS_PEER")
    interface = config.get("INTERFACE")
    domain_id = config["DOMAIN_ID"]
    try:
        if interface:
            ChannelFactoryInitialize(domain_id, interface, peer_address=peer_address)
        else:
            ChannelFactoryInitialize(domain_id, peer_address=peer_address)
    except TypeError:
        if peer_address:
            raise TypeError(
                "this unitree_sdk2py has no peer_address support; "
                "unset dds.dds_peer (GEAR Tailscale-only). Local sim uses interface lo."
            ) from None
        if interface:
            ChannelFactoryInitialize(domain_id, interface)
        else:
            ChannelFactoryInitialize(domain_id)
