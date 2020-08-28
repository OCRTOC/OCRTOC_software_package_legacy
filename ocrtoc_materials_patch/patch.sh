#!/bin/bash
cd $0
echo "Copy files"
cp -r models /root/ocrtoc_materials/
cp -r scenes /root/ocrtoc_materials/
cp objects.csv /root/ocrtoc_materials/
echo "Done"
