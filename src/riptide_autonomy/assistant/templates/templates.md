# BT Assistant Templates
The BT assistant takes the templates in this directory and fills in the holes to create new header and source files. Spots in the files that need to be filled in with information are specified by text markers that look like "\<X\>", where X is the index of the marker. Markers with like numbers are filled in with the same information. The templates in this directory should be filled in with the following markers:

- actionnode_hpp_template AND nonactionnode_hpp_template
  - <1>: The node name, title-cased. Should be identical to the HPP file name
  - <2>: The superclass of the node.
- plugin_registrator_template
  - <1>: All headers needed by the registrator
  - <2>: The calls needed to register the nodes
- test_template
  - <1>: The node name, title-cased.
