#include "riptide_autonomy/autonomy_health.hpp"

//
// AutonomyUndefinedIssue
//

AutonomyUndefinedIssue::AutonomyUndefinedIssue(const std::string& file, tinyxml2::XMLElement *node)
 : AutonomyIssue(ISSUE_ERROR, file, node->GetLineNum(), "AutonomyUndefinedIssue",
                  "Node " + std::string(node->Name()) + " is undefined")
 { }


HealthError AutonomyUndefinedIssue::fix()
{
   return HealthError(false, "");
}


//
// AutonomyNodeIssueDetector
//

AutonomyNodeIssueDetector::AutonomyNodeIssueDetector(
    tinyxml2::XMLElement *node,
    const std::string& file,
    std::shared_ptr<const BT::BehaviorTreeFactory> factory,
    const NodeManifests& palette,
    const std::vector<std::string>& blackboardDefinitions)
 : _node(node),
   _fileName(file),
   _factory(factory),
   _palette(palette),
   _blackboardDefs(blackboardDefinitions) { }


HealthError AutonomyNodeIssueDetector::detect()
{
   std::string nodeName = _node->Name(); //should exist

   //does it exist in the manifests
   if(_factory->manifests().count(nodeName) == 0)
   {
      addIssue(std::make_shared<AutonomyUndefinedIssue>(_fileName, _node));
      return HealthError(true, "Aborted due to previous errors");
   }

   if(_factory->builtinNodes().count(nodeName) == 0 && _palette.count(nodeName) == 0)
   {
      addIssue(
         std::make_shared<UnfixableAutonomyIssue>(
            ISSUE_ERROR,
            _fileName,
            _node->GetLineNum(),
            "ModelError",
            "Node " + nodeName + " is not present in the TreeNodesModel"));

      return HealthError(true, "Aborted due to previous errors");
   }

   //now process individual port values for issues
   BT::PortsList btPorts = _factory->manifests().at(nodeName).ports;

   // check UWRT port information if able to
   std::map<std::string, UwrtPortNecessity> portNecessities;

   if(UwrtNodesManifest::hasInformationForNode(nodeName))
   {
      UwrtPortInformation uwrtPorts = UwrtNodesManifest::lookupInformationByNodeName(nodeName);
      for(UwrtPort port : uwrtPorts)
      {
         portNecessities.insert({ std::string(port.name()), port.necessity() });
      }
   }         


   //check for bad blackboard refs (this does not require uwrt ports so it is done in another loop)
   for(auto pair : btPorts)
   {
      //name and value
      std::string portName = pair.first;
      const char *portValue = _node->Attribute(portName.c_str());

      //                                                 super secret hack
      UwrtPortNecessity necessity = (portName.at(0) == '_' ? PORT_OPTIONAL : PORT_REQUIRED);
      if(portNecessities.count(portName) > 0)
      {
         necessity = portNecessities.at(portName);
      }

      // necessity. If required then the value must be provided
      if((!portValue || std::string(portValue).empty()) && necessity == PORT_REQUIRED)
      {
         addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
               ISSUE_ERROR,
               _fileName,
               _node->GetLineNum(),
               "RequiredPortError",
               nodeName + " missing value for required port \"" + portName + "\""));
         
         continue;
      }

      //bad blackboard ref
      if(portValue && BT::TreeNode::isBlackboardPointer(portValue))
      {
         //now check that blackboard reference is good
         std::string targetPointer = std::string(BT::TreeNode::stripBlackboardPointer(portValue));
         if(std::find(_blackboardDefs.begin(), _blackboardDefs.end(), targetPointer) == _blackboardDefs.end())
         {
            addIssue(
               std::make_shared<UnfixableAutonomyIssue>(
                  ISSUE_ERROR,
                  _fileName,
                  _node->GetLineNum(),
                  "PortError",
                  "Blackboard variable " + targetPointer + " may not be defined yet"));
            
            continue;
         }
      }
   }   

   return HealthError(false, "");
}
