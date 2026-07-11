# UCI OMS-JSON seam contract

This is a deliberately small, mechanically XSD-shaped UCI 2.5 contract for
the ActionCommand request and ActionCommandStatus requirement legs.  It is not
a general UCI model.  The OMS-JSON backend accepts this shape only.

Proto field names are normal `snake_case`.  The backend maps them to UCI
element names deterministically: each word becomes PascalCase, with `id` and
`uuid` rendered as `ID` and `UUID`.  Thus `system_id` becomes `SystemID` and
`command_id` becomes `CommandID`; no hand-cased wire names are embedded in the
IDL.  A message is wrapped once by its global-element name (`ActionCommand` or
`ActionCommandStatus`), repeated fields become arrays, and optional MissionID
and ServiceID are omitted when absent.

The provided/consumed request ports use the standard Create/Read/Update/Cancel
grammar. Their request topic is `mission.action_command.request`; their
requirement topic is `mission.action_command.requirement`. `Cancel` receives
an `Identifier`; its `id` is the CommandID UUID string.
