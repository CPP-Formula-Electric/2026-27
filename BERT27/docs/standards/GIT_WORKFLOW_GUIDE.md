# Git Workflow Guide


1. **Make a branch.** Never work on `main`.
   ```
   git switch -c fix/bms-can-messages
   ```
   Name it `<type>/<board>-<what>`, such as `feat/vcu-regen-limit`.
2. **Write the code.** Follow [CODE_GUIDE.md](CODE_GUIDE.md).
3. **Commit** follow [VERSION_GUIDE.md](VERSION_GUIDE.md) and the commit messages format.
4. **Update the required documents.**
5. **Push and open a pull request.**

## Commit messages

Format: `type(board): what you changed`

| Type | Use it for |
|---|---|
| `feat` | a new feature |
| `fix` | a bug fix |
| `cal` | changing a calibration value or threshold |
| `docs` | docs only |
| `refactor` | moving or renaming code without changing what it does |

For titles: 

Good: `fix(bms): read all CAN messages during temperature sweep`

Bad: `fixed stuff`, `TTT day 2`

Within the comment body the more details you add the better but don't add too much as thats what the explanation docs are for. 

If there is anything that still needs to be completed add to the TODO checklist in the readme for that board/folder.

## Who reviews (Will be finalized once proper team structuring is in place)

**TO DO**

## Changelog 

**TO DO**