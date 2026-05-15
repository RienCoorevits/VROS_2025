# Drawings

Store serial-streamable drawing files in this folder.

The recommended format matches the SD instruction format already used by the firmware:

```text
type	absolute
mode	segmented
adjustment	none
move	21,31
move	25,31
move	25,35
move	21,35
move	21,31
```

Use a literal tab character between the command and the value.

Supported streamed instructions:

- `move<TAB>scan,feed`
- `type<TAB>absolute|relative`
- `mode<TAB>segmented|movePen`
- `adjustment<TAB>none|largeSin|complexSin|noise`

Lines starting with `#` and blank lines are ignored by the host streaming script.
