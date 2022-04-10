#!/bin/bash

packageName=$1
packageName="${packageName,,}"

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
find "${SCRIPTPATH}/.." -type f -exec sed -i -e 's/template_/'"$packageName"'_/g' {} \;

rm -rf ${SCRIPTPATH}/../.git

mv ${SCRIPTPATH}/../src/template_control.cpp ${SCRIPTPATH}/../src/${packageName}_control.cpp 
mv ${SCRIPTPATH}/../src/template_navigation.cpp ${SCRIPTPATH}/../src/${packageName}_navigation.cpp 
mv ${SCRIPTPATH}/../src/template_guidance.cpp ${SCRIPTPATH}/../src/${packageName}_guidance.cpp 
mv ${SCRIPTPATH}/../src/template_fsm.cpp ${SCRIPTPATH}/../src/${packageName}_fsm.cpp 

mv ${SCRIPTPATH}/../launch/template_SIL.launch ${SCRIPTPATH}/../launch/${packageName}_SIL.launch 

mv ${SCRIPTPATH}/../../template_gnc ${SCRIPTPATH}/../../${packageName}_gnc 

echo "Updated name of package to $packageName""_gnc"
