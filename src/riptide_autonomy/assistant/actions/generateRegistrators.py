import os
from util import categorizedGlob, fileNameNoExt, createFileFromTemplate, info, autonomySrcLocation, autonomyIncludeLocation

def onGenerateRegistrators(args, autonomyRootLoc: str):
    location = args.directory
    
    #resolve general node file names    
    generalActionFiles, generalConditionFiles, generalDecoratorFiles = categorizedGlob(autonomyIncludeLocation(autonomyRootLoc), "*.hpp")
        
    #creates a single registrator
    def createRegistrator(generalFiles: 'list[str]', location: str):
        registrations = ""
        headers = ""
        for file in generalFiles:
            # perform general registration on all other hpps
            registrations += "    registerUwrtNode<{0}>(\"{0}\", factory);\n".format(fileNameNoExt(file))
            
            #figure out node type (its in the name between the second to last and last dots)
            lastDelimiter = file.rfind("/")
            secondLastDelimiter = file.rfind("/", 0, lastDelimiter - 1)
                            
            #not checking dots because files need to have two to be globbed
            nodeType = file[secondLastDelimiter + 1 : lastDelimiter]
            headers += "#include \"riptide_autonomy/{}/{}\"\n".format(nodeType, os.path.basename(file))

        templatePath = "{}/assistant/templates/plugin_registrator_template".format(autonomySrcLocation(autonomyRootLoc))
        createFileFromTemplate(templatePath, location, [headers, registrations])
    
    createRegistrator(generalActionFiles, "{}/registerActions.cpp".format(location))
    createRegistrator(generalConditionFiles, "{}/registerConditions.cpp".format(location))
    createRegistrator(generalDecoratorFiles, "{}/registerDecorators.cpp".format(location))
    
    info(args, "Generated registrators in directory {}".format(os.path.abspath(location)))
