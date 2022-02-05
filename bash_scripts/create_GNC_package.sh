#!/bin/bash

packageName=$1
packageName="${packageName,,}"

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
find "${SCRIPTPATH}/.." -type f -exec sed -i -e 's/mydrone_/'"$packageName"'_/g' {} \;


mv ${SCRIPTPATH}/../src/mydrone_control.cpp ${SCRIPTPATH}/../src/${packageName}_control.cpp 
mv ${SCRIPTPATH}/../src/mydrone_navigation.cpp ${SCRIPTPATH}/../src/${packageName}_navigation.cpp 
mv ${SCRIPTPATH}/../src/mydrone_guidance.cpp ${SCRIPTPATH}/../src/${packageName}_guidance.cpp 
mv ${SCRIPTPATH}/../src/mydrone_fsm.cpp ${SCRIPTPATH}/../src/${packageName}_fsm.cpp 

mv ${SCRIPTPATH}/../../mydrone_gnc ${SCRIPTPATH}/../../${packageName}_gnc 
mv ${SCRIPTPATH}/../launch/mydrone_SIL.launch ${SCRIPTPATH}/../launch/${packageName}_SIL.launch 

echo "Updated name of package to $packageName""_gnc"



