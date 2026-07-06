# Review Record: <type> — <item>

| Field | Value |
|-------|-------|
| Date | YYYY-MM-DD |
| Type | HLR review / LLR review / Design review / Code review / Trace review / QA audit |
| Item(s) under review | e.g. `HLR.md` PCL.058–PCL.063, or `src/pcl_codec_registry.c` |
| Revision reviewed | git commit hash |
| Checklist used | `checklists.md` section + its git revision |
| Author of item | name |
| Reviewer(s) | name(s) |
| Independent of author | yes / no |

## Findings

| # | Checklist item | Finding | Severity (major/minor/observation) | Disposition |
|---|----------------|---------|-----------------------------------|-------------|
| 1 | | | | fixed in `<commit>` / accepted deviation (register ref) / open (issue #) |

## Result

- [ ] Accepted
- [ ] Accepted with findings dispositioned as above
- [ ] Rejected — rework and re-review required

## Notes

<optional context, sampling scope, follow-ups>

---
After completing: save as `YYYY-MM-DD_<type>_<item>.md` in this directory and
add a row to the index in `README.md` in the same commit.
