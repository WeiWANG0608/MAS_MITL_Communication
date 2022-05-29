#!/bin/bash

cd 

cd uppaal64-4.1.26

cd demo/ModelDemo

ant clean jar

cd ../../

java -cp uppaal.jar:lib/model.jar:demo/ModelDemo/dist/ModelDemo.jar:lib/json-simple-1.1.1.jar MITL_verification hardcoded
