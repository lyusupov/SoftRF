
# set(COMPONENT_SRCS "./AXP202X_Library/src/axp20x.cpp")
# set(COMPONENT_ADD_INCLUDEDIRS "./AXP202X_Library/src")
# register_component()

idf_component_register(SRCS "src/axp20x.cpp"
                    INCLUDE_DIRS "src"
                    )