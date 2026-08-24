# Git and GitHub — a guide for this project

Written for people who have never used git. If you have, skip to
[Branch naming](#4-branch-naming--the-rule-for-this-project) and
[Rules for this project](#10-rules-for-this-project) — those are the parts specific
to us.

---

## 1. What git actually is

**Git** records snapshots of this folder over time. Every snapshot (a *commit*)
remembers what changed, who changed it, and why. Nothing is ever really lost, so
you can experiment without fear — this is the whole point.

**GitHub** is a website that stores a copy of that history so several people can
share it. Git is the tool on your computer; GitHub is the copy in the cloud.

Three places your work lives:

| Where | What it means |
|---|---|
| **Working tree** | The files you actually edit. What you see in your editor |
| **Staging area** | Changes you have marked as "part of my next commit" (`git add`) |
| **Commit history** | Snapshots you have saved (`git commit`), local until you `git push` |

The most common beginner surprise: **committing does not upload anything.** A
commit is local. `git push` is what sends it to GitHub.

---

## 2. One-time setup

Covered in [README.md](README.md) section 1 — set your name and email, make an
SSH key, and clone. Do it on **both** the robot and your workstation; they are
separate computers and each needs its own key.

---

## 3. The everyday loop

This is 95% of what you will ever do:

```bash
# 1. Start from the latest shared code
git checkout mycobot_main
git pull

# 2. Make a branch for what you are about to do
git checkout -b fix/gripper_never_opens

# 3. ... edit files ...

# 4. See what you changed
git status
git diff

# 5. Save a snapshot
git add src/swarm_pkg/src/scripts/gripper_test.py
git commit -m "Fix gripper never opening: wrong serial port"

# 6. Send it to GitHub
git push -u origin fix/gripper_never_opens
```

Then open a **Pull Request** on GitHub (see §6).

`git push -u origin <branch>` is only needed the *first* time you push a new
branch. After that, plain `git push` works.

### Commit early, commit often

A commit is cheap. Ten small commits that each do one thing are far easier to
review — and to undo — than one commit containing a week of work.

Write messages that say **why**, not what. `git diff` already shows what.

```
Bad:   "update file"          "changes"        "fix"
Good:  "Use /dev/ttyAMA0: serial0 is the mini UART on a Pi 4"
```

---

## 4. Branch naming — the rule for this project

A **branch** is a parallel line of work. You make one, do your thing, and it gets
merged back. Branches keep half-finished work off everyone else's machine.

Every branch you create must start with one of these three prefixes:

| Prefix | Use it when | Real examples from this repo |
|---|---|---|
| `feature/` | Adding something new | `feature/april_tags`, `feature/camera_integration`, `feature/hardware_bridge` |
| `fix/` | Repairing something broken | `fix/mycobot_build_errors`, `fix/project_structure` |
| `test/` | Experiments, characterization, measurement runs | `test/workability_annulus` |

Format rules:

- **Lowercase.** `feature/thing`, never `Feature/Thing`
- **Underscores or hyphens between words**, never spaces
- **Letters, digits, `_`, `-` and the one `/` only.** No `&`, no `#`, no quotes
- **Say what it is about**, not who did it. `feature/gripper_offset`, not `feature/rhishi_branch`

```bash
git checkout -b feature/block_colour_tuning     # good
git checkout -b feature/Block Colour Tuning     # breaks — spaces
git checkout -b gripper-stuff                   # no prefix, rejected in review
```

### Long-lived branches — do not create these, just use them

| Branch | What it is |
|---|---|
| `mycobot_main` | **The arm's main line. Branch from here, PR back into here** |
| `myagv_main` | The AGV line. A different robot |
| `main` | Long out of date. Do not build from it or target it |

### You will see branches that break these rules

The repo has older branches named `features/...` (plural), `tests/...`,
`upgrade/pick_place.py`, and one with no prefix at all. One is even called
`features/gripper&camerascripts` — the `&` means bash treats it as a background
job unless you quote it:

```bash
git checkout "features/gripper&camerascripts"   # quotes required
```

**These are drift, not examples.** Use `feature/`, `fix/`, `test/`.

---

## 5. Working across two machines

You will edit on the workstation and need it on the robot, or the reverse. Git is
how the code crosses — not a USB stick, not scp.

```bash
# On the machine where you made the change
git push

# On the other machine
git pull
```

**After pulling on either machine, rebuild.** Git updates source files; it does
not update what you built:

```bash
colcon build --packages-skip mycobot_hardware   # workstation
colcon build                                    # robot
source install/setup.bash
```

**If the pull changed a DDS peer IP, you must rebuild `swarm_network`** or the
change has no effect at all — `CYCLONEDDS_URI` points into `install/`, not into
your source tree. See README section 6.

---

## 6. Pull requests

A **Pull Request** (PR) asks for your branch to be merged into `mycobot_main`.
It is also where review happens.

1. Push your branch
2. Go to the repo on GitHub — it will offer a "Compare & pull request" button
3. Set the target branch to **`mycobot_main`** (not `main`)
4. Describe what you changed and **how you tested it**. On this project that
   means saying whether it ran on the real arm or only on the workstation
5. Ask someone to review it

Keep working after opening a PR by committing and pushing to the same branch —
the PR updates itself.

---

## 7. Command cheat sheet

| Command | What it does |
|---|---|
| `git status` | What have I changed? **Run this constantly** |
| `git diff` | Show the actual line-by-line changes, unstaged |
| `git diff --staged` | Show what is about to be committed |
| `git add <file>` | Stage one file. `git add -p` stages piece by piece |
| `git commit -m "..."` | Save a snapshot locally |
| `git push` | Upload commits to GitHub |
| `git pull` | Download and merge others' commits |
| `git log --oneline -10` | Last 10 commits, one line each |
| `git branch` | List local branches; `*` marks the current one |
| `git checkout <branch>` | Switch branches |
| `git checkout -b <branch>` | Create a branch and switch to it |
| `git checkout -- <file>` | **Throw away** your changes to one file |
| `git stash` / `git stash pop` | Park changes temporarily, then bring them back |

---

## 8. When it goes wrong

Almost nothing in git is unrecoverable. Ask before doing anything drastic.

### "Your branch is behind" / push rejected
Someone else pushed first. Get their work, then push:
```bash
git pull
git push
```

### Merge conflict
Two people changed the same lines. Git marks them in the file:
```
<<<<<<< HEAD
your version
=======
their version
>>>>>>> mycobot_main
```
Edit the file so it reads correctly — **delete all three marker lines** — then:
```bash
git add <file>
git commit
```

### I committed to the wrong branch
The commit is not pushed yet, so this is easy:
```bash
git log --oneline -1              # copy the commit hash
git checkout -b feature/correct_branch
git checkout mycobot_main
git reset --hard HEAD~1           # removes it from the wrong branch
```
`reset --hard` discards work permanently. Make sure the commit really is on the
new branch (`git log`) before running it.

### I need to undo my last commit but keep the changes
```bash
git reset --soft HEAD~1
```

### "detached HEAD"
You checked out a commit instead of a branch. Get back with:
```bash
git checkout mycobot_main
```

### git asks for a username and password every time
You cloned over HTTPS instead of SSH. Fix the remote:
```bash
git remote set-url origin git@github.com:kandge1/swarm_project.git
```

### I cannot push — "permission denied"
You have not been added as a collaborator, or your SSH key is not on GitHub.
Test with `ssh -T git@github.com`.

---

## 9. What not to commit

The [.gitignore](.gitignore) already excludes the big offenders — `build/`,
`install/`, `log/`, `__pycache__/`, `*.pyc`. **Never commit those**; they are
generated, they are large, and they differ per machine.

Before every commit, run `git status` and look at what you are about to include.
If you see `build/` or `install/` in the list, stop — something is wrong with
your ignore rules, not with your intent.

Also avoid committing: camera stills and debug images unless they are evidence
worth keeping, calibration logs from throwaway runs, and anything containing a
password or key.

---

## 10. Rules for this project and general software development with git

1. **Never commit directly to `mycobot_main` or `main`.** Branch, then PR.
2. **Never `git push --force` a branch anyone else uses.** It deletes their work
   with no warning. On your own unshared branch it is fine.
3. **Pull before you start**, not after you finish. Conflicts are much easier to
   resolve before you have written 200 lines on top of stale code.
4. **One branch per task.** A branch that fixes the gripper and adds a camera
   feature cannot be reviewed or reverted cleanly.
5. **Say how you tested it** in the PR — real arm, or workstation only.
6. **If you break something on the robot, say so immediately.** The arm is shared
   hardware and a silent breakage costs the next group their session.

---

## 11. Learning more

- `git help <command>` — e.g. `git help commit`
- [Git Book, chapters 1-3](https://git-scm.com/book/en/v2) — free, genuinely good
- [GitHub Docs](https://docs.github.com/en/get-started)
