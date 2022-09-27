# BT Assistant Templates
The BT assistant takes the templates in this directory and fills in the holes to create new header and source files. Spots in the files that need to be filled in with information are specified by text markers that look like "\<X\>", where X is the index of the marker. Markers with like numbers are filled in with the same information. The templates in this directory should be filled in with the following markers:

- node_header_template:
  - <1>: the name of the class, capitalized
  - <2>: the name of the class/file
  - <3>: the name of the parent class.
- node_src_template:
  - <1>: the name of the class/file
- autonomy_header_template:
  - <1>: the #include's for all autonomy header files
- node_registrator_template:
  - <1>: all code needed to register custom nodes
