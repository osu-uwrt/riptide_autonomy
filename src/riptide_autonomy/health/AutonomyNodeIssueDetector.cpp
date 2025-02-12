#include "riptide_autonomy/autonomy_health.hpp"

AutonomyNodeIssueDetector::AutonomyNodeIssueDetector(
    tinyxml2::XMLElement *node,
    const std::string& file,
    const BT::BehaviorTreeFactory& factory,
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
   if(_factory.manifests().count(nodeName) == 0)
   {
      addIssue(std::make_shared<AutonomyUndefinedIssue>(_node));
   }

   if(!UwrtNodesManifest::hasInformationForNode(nodeName))
   {
      addIssue(
         std::make_shared<UnfixableAutonomyIssue>(
            ISSUE_ERROR,
            "",
            0,
            "CodeError",
            "Node with name " + nodeName + " is not known by the UWRT nodes manifest"));
         
      return HealthError(true, "Aborted due to previous errors");
   }

   UwrtPortInformation ports = UwrtNodesManifest::lookupInformationByNodeName(nodeName);

   //now iterate through ports. Make sure the required ones are populated and that bb refs are good
   for(UwrtPort port : ports)
   {
      std::string portName(port.name());

      UwrtPortNecessity necessity = port.necessity();

      const char *portValue = _node->Attribute(portName.c_str());

      if(!portValue && necessity == PORT_REQUIRED)
      {
         addIssue(
            std::make_shared<UnfixableAutonomyIssue>(
               ISSUE_ERROR,
               _fileName,
               _node->GetLineNum(),
               "PortError",
               nodeName + " missing value for required port " + portName));
         
         continue;
      }

      if(BT::TreeNode::isBlackboardPointer(portValue))
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
}
