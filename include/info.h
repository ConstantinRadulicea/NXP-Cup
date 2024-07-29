/*
Trivial Variables: i,n,c,etc... (Only one letter. If one letter isn't clear, then make it a Local Variable)
Local Variables: camelCase
Global Variables: g_camelCase
Const Variables: ALL_CAPS
Pointer Variables: add a p_ to the prefix. For global variables it would be gp_var, for local variables p_var, for const variables p_VAR. If far pointers are used then use an fp_ instead of p_.
Structs: ModulePascalCase (Module = full module name, or a 2-3 letter abbreviation, but still in PascalCase.)
Struct Member Variables: camelCase
Enums: ModulePascalCase
Enum Values: ALL_CAPS
Public Functions: ModulePascalCase
Private Functions: PascalCase
Macros: PascalCase
*/