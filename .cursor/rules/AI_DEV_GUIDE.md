# Developing on g1-simulacrum with an AI coding assistant

Practical guide for Cursor on this repo, and for keeping `.cursor/rules/`
sharp. Read once; skim before a session.

---

## 1. What the rules are

The agent starts every chat with zero memory. `.cursor/rules/` injects
context. These files live in **this git repo** (open it as the Cursor
workspace when working here, or `@` the rules). They do not apply to
`ws_weltmodelle` or other trees under `/home/aj/code`.

| File | Activation | Purpose |
|------|------------|---------|
| `00-project-context.mdc` | **Always** | Identity, layout, scope, working agreement |
| `30-git.mdc` | **Always** | Single repo, yojuna identity, origin |
| `31-git-hygiene.mdc` | **Always** | Logical commits; `feat/` `fix/` `tooling/` `docs/` |
| `40-docker-tooling.mdc` | **Always** | `run.sh` only; no host venv |
| `10-python.mdc` | **Auto** (`**/*.py`) | Config-driven Python, naming, tests |

Precedence: Team > Project (`.cursor/rules/`) > User rules. Agent Chat only.

**Token hygiene:** keep always-on files small. Push detail into glob-scoped
files. Do not paste architecture drafts into always-on rules — point at
`ARCHITECTURE.md` / `wiki/` / `docs/`.

Dropped from weltmodelle on purpose: JEPA research discipline, experiment
campaigns, parent/submodule git.

---

## 2. Rules grow from mistakes

Every time the agent does something you undo, encode the fix. Commit rule
changes like code.

Good triggers:

- Edited `ws_simulacra/` or `/home/aj/code` as if that were the git root.
- Used haverboecker / global git identity on a commit.
- Created `.venv` or `pip install` on the host.
- Ran `python` on the host instead of `docker/run.sh`.
- Used `docker compose run` / `down` instead of `./run.sh`.
- Wired GEAR-SONIC into core instead of keeping a sensorized G1 package.
- Mixed docs / docker / code in one commit, or `git add -A`.

---

## 3. Working with the agent

**Plan first, then diff.** Beyond a one-file tweak, ask for a short plan
and approve it before code.

**Specs before implementation.** `ARCHITECTURE.md` is normative for design.
`wiki/` is the cited hardware fact book. Numbers in code must match both.
Implement that core (pinned MJCF includes, PD/passthrough, sensors). Do not
add SONIC, ROS2, or RoboCasa into core. `SonicBridge` was deleted; do not
bring it back.

**Ask, don’t guess.** Joint order, frames, MuJoCo depth units, mount poses →
read `wiki/`, then ask if the wiki has no citation.

**Plain language.** Short sentences. Jargon is for named quantities
(Mid-360, D435i, `qpos`), not style.

**Keep the tree lean.** Prefer deleting prototype lies over adding adapters
that hide them.

---

## 4. Checklist before trusting a run

- [ ] Ran inside `docker/run.sh` (including tests) — never host Python.
- [ ] Knobs from YAML / Pydantic — nothing new hardcoded.
- [ ] Did not add a pip dependency without asking.
- [ ] Image rebuild after Dockerfile edits (`./run.sh up --build`).
- [ ] Commit is `yojuna` / `datamongeraami@gmail.com`, not haverboecker.
