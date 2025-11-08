# Robospace Git Workflow - Best Practices

## Current Situation

You have a feature branch with design documents:
- Branch: `claude/robospace-architecture-design-011CUue856DzJRvLBw9Sja17`
- Contains: Design documents, analysis, planning

**Recommended Action:** Merge to main, then create new implementation branches

---

## Recommended Workflow

### Step 1: Merge Design Documents to Main

```bash
# Make sure you're on the design branch
git checkout claude/robospace-architecture-design-011CUue856DzJRvLBw9Sja17

# Make sure everything is committed and pushed
git status  # Should be clean
git log --oneline -5  # Verify commits

# Switch to main and merge
git checkout main
git pull origin main  # Get latest
git merge claude/robospace-architecture-design-011CUue856DzJRvLBw9Sja17

# Push to remote
git push origin main

# Optional: Delete the old branch (keep remote for history)
# git branch -d claude/robospace-architecture-design-011CUue856DzJRvLBw9Sja17
```

**Why?**
- ✅ Design phase complete - documents ready for implementation
- ✅ Clean separation: design → implementation
- ✅ Main branch has latest stable documents
- ✅ New feature branches start from clean main

---

### Step 2: Create Feature Branch for Phase 1

```bash
# Make sure you're on main
git checkout main
git pull origin main  # Ensure up-to-date

# Create new feature branch for Phase 1
git checkout -b feature/phase1-math-library

# Alternative naming options:
# git checkout -b feature/math-library
# git checkout -b feature/foundation
# git checkout -b impl/phase1-math
```

**Branch Naming Convention:**
- `feature/<feature-name>` - new features
- `fix/<bug-name>` - bug fixes
- `docs/<doc-name>` - documentation only
- `refactor/<module>` - code refactoring
- `test/<test-name>` - test additions

---

## Implementation Workflow (Phase 1 Example)

### Option 1: Single Feature Branch for Entire Phase 1

**Pros:** Simple, linear history
**Cons:** Long-lived branch, harder to review

```bash
# One branch for all Phase 1 work
git checkout -b feature/phase1-foundation

# Work on math library
git add core/math/
git commit -m "Implement Transform and Rotation classes"

# Work on Python bindings
git add python/bindings/
git commit -m "Add Python bindings for math module"

# Continue until Phase 1 complete...
git push origin feature/phase1-foundation

# When done, merge to main
git checkout main
git merge feature/phase1-foundation
git push origin main
```

### Option 2: Sub-Feature Branches (RECOMMENDED)

**Pros:** Smaller PRs, easier review, parallel work possible
**Cons:** More branch management

```bash
# Create sub-branches for each major component

# Math library
git checkout main
git checkout -b feature/math-library-cpp
# ... implement Transform, SE3, etc.
git push origin feature/math-library-cpp
# Merge to main when done

# Python bindings
git checkout main
git checkout -b feature/math-library-python
# ... implement bindings
git push origin feature/math-library-python
# Merge to main when done

# Robot model
git checkout main
git checkout -b feature/robot-model
# ... implement Joint, Link, URDF parser
git push origin feature/robot-model
# Merge to main when done

# And so on...
```

**Branch Structure:**
```
main
├── feature/math-library-cpp          (Milestone 1)
├── feature/math-library-python       (Milestone 1)
├── feature/robot-model               (Milestone 2)
├── feature/forward-kinematics        (Milestone 2)
├── feature/item-pattern              (Milestone 3)
└── feature/phase1-ci-cd              (Milestone 4)
```

### Option 3: Hybrid Approach (BEST FOR THIS PROJECT)

**Recommended for robospace:** Combine both approaches

```bash
# Create main Phase 1 branch
git checkout -b feature/phase1-foundation

# Create sub-branches FROM phase1 branch for specific tasks
git checkout feature/phase1-foundation
git checkout -b feature/phase1-math-cpp

# Work on math library
git commit -m "Add SE3 class"
git commit -m "Add se3 class"
git commit -m "Add exp_se3 and log_SE3"

# Merge back to phase1-foundation
git checkout feature/phase1-foundation
git merge feature/phase1-math-cpp

# Push phase1-foundation periodically
git push origin feature/phase1-foundation
```

**Benefits:**
- ✅ Can work on multiple components in parallel
- ✅ Easy to cherry-pick or reorder changes
- ✅ Main branch stays clean
- ✅ Phase 1 branch is the integration point

---

## Commit Message Best Practices

### Format

```
<type>: <subject>

<body (optional)>

<footer (optional)>
```

### Types
- `feat:` - New feature
- `fix:` - Bug fix
- `docs:` - Documentation only
- `style:` - Code style (formatting, no logic change)
- `refactor:` - Code refactoring
- `test:` - Adding tests
- `chore:` - Build, CI, dependencies

### Examples

**Good Commit Messages:**
```bash
git commit -m "feat: implement SE3 Lie group class with exp/log maps"

git commit -m "feat: add Python bindings for Transform class

- Expose all Transform methods to Python
- Add operator overloading for * and ==
- Enable Eigen to NumPy conversion
- Add comprehensive docstrings"

git commit -m "test: add unit tests for exponential map

- Test exp(log(g)) = g for random SE3 elements
- Test log(exp(xi)) = xi for random twists
- Add performance benchmarks"

git commit -m "docs: add API documentation for math module"

git commit -m "fix: correct Rodrigues formula in exp_se3 for small angles"
```

**Bad Commit Messages:**
```bash
git commit -m "fixed stuff"
git commit -m "WIP"
git commit -m "updates"
git commit -m "asdfasdf"
```

---

## Pull Request Workflow (if using PRs)

### For Small Team / Solo Development

**Option A: Direct Merge (Simple)**
```bash
# Work on feature branch
git checkout feature/math-library
# ... commits ...
git push origin feature/math-library

# When ready, merge directly
git checkout main
git merge feature/math-library
git push origin main

# Delete branch
git branch -d feature/math-library
```

**Option B: PR for Review (Recommended)**
```bash
# Push feature branch
git push origin feature/math-library

# Create PR on GitHub
# Title: "Add math library with Lie algebra support"
# Description:
#   - Implements Transform, SE3, se3 classes
#   - Adds exponential/logarithm maps
#   - Includes 50+ unit tests
#   - Closes #123

# Review, approve, merge via GitHub UI
# (Or use `gh pr create` with GitHub CLI)
```

---

## Recommended Workflow for Robospace

Based on project characteristics:
- Solo/small team development
- Long-running project (18 months)
- Clear phase structure

### Phase-Based Branch Strategy

```
main (always deployable)
  │
  ├── feature/phase1-foundation  (2 months)
  │   ├── feature/phase1-math
  │   ├── feature/phase1-robot
  │   ├── feature/phase1-bindings
  │   └── feature/phase1-tests
  │
  ├── feature/phase2-kinematics  (2 months)
  │   ├── feature/phase2-ik
  │   └── feature/phase2-planning
  │
  ├── feature/phase3-web         (2 months)
  │   ├── feature/phase3-backend
  │   └── feature/phase3-frontend
  │
  ... and so on
```

### Daily Workflow

```bash
# Morning: Start work
git checkout feature/phase1-foundation
git pull origin feature/phase1-foundation

# Create task branch (optional, for complex tasks)
git checkout -b task/implement-se3-class

# Work and commit frequently
git add core/math/se3.cpp
git commit -m "feat: implement SE3 multiplication"

git add core/math/se3.cpp
git commit -m "feat: add SE3::inverse() method"

# Push to remote daily (backup)
git push origin task/implement-se3-class

# Merge back when task complete
git checkout feature/phase1-foundation
git merge task/implement-se3-class
git push origin feature/phase1-foundation

# Delete task branch
git branch -d task/implement-se3-class
```

---

## Merging Strategy

### Fast-Forward vs. Merge Commit

**Fast-Forward (Default, Clean History):**
```bash
git checkout main
git merge --ff-only feature/math-library  # Fails if can't fast-forward
```

**Merge Commit (Explicit, Preserves Branch History):**
```bash
git checkout main
git merge --no-ff feature/math-library  # Always creates merge commit
```

**Recommendation:** Use `--no-ff` for feature branches to preserve history

```bash
# Good practice
git merge --no-ff feature/phase1-foundation
```

### Rebase vs. Merge

**Rebase (Linear History):**
```bash
# Update feature branch with main changes
git checkout feature/math-library
git rebase main
git push --force-with-lease origin feature/math-library
```

**Merge (Preserve History):**
```bash
git checkout feature/math-library
git merge main
git push origin feature/math-library
```

**Recommendation:**
- Rebase local branches before merging to main (clean history)
- Never rebase public/shared branches
- Use `git pull --rebase` for updating your branch

---

## Tagging Milestones

```bash
# After merging Phase 1
git checkout main
git tag -a v0.1.0-phase1 -m "Phase 1 complete: Math library and foundation"
git push origin v0.1.0-phase1

# After merging Phase 2
git tag -a v0.2.0-phase2 -m "Phase 2 complete: Kinematics and planning"
git push origin v0.2.0-phase2
```

**Tag Naming:**
- `v0.1.0-phase1` - Phase milestones
- `v1.0.0` - First production release
- `v1.1.0` - Minor version (new features)
- `v1.1.1` - Patch version (bug fixes)

---

## Summary: Your Next Steps

### 1. Merge Current Design Branch
```bash
git checkout main
git merge claude/robospace-architecture-design-011CUue856DzJRvLBw9Sja17
git push origin main
```

### 2. Create Phase 1 Branch
```bash
git checkout -b feature/phase1-foundation
git push -u origin feature/phase1-foundation
```

### 3. Start Implementation
```bash
# Create directory structure
mkdir -p core/math core/model python/bindings tests/cpp

# Start with math library
# ... code ...

# Commit frequently
git commit -m "feat: add project structure for Phase 1"
git commit -m "feat: implement Transform class"
git push origin feature/phase1-foundation
```

### 4. Milestone Workflow
```bash
# When Milestone 1 complete (math library)
git commit -m "feat: complete math library with Lie algebra support"
git tag -a milestone-1 -m "Math library complete"
git push origin feature/phase1-foundation --tags

# Continue with Milestone 2, 3, 4...
```

### 5. Phase 1 Complete
```bash
# Merge to main
git checkout main
git merge --no-ff feature/phase1-foundation
git tag -a v0.1.0-phase1 -m "Phase 1 foundation complete"
git push origin main --tags

# Start Phase 2
git checkout -b feature/phase2-kinematics
```

---

## Quick Reference

| Task | Command |
|------|---------|
| Create feature branch | `git checkout -b feature/name` |
| Switch branches | `git checkout branch-name` |
| Commit changes | `git commit -m "type: message"` |
| Push to remote | `git push origin branch-name` |
| Update from main | `git merge main` or `git rebase main` |
| Merge to main | `git checkout main && git merge --no-ff feature/name` |
| Create tag | `git tag -a v1.0.0 -m "message"` |
| Push tags | `git push origin --tags` |

---

**You're all set!** 🚀 Merge the design docs to main, create your Phase 1 branch, and let's start coding!
