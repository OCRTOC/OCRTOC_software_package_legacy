#!/bin/bash

OSS_BUCKET_DOMAIN="https://ocrtoc-public.oss-cn-hangzhou.aliyuncs.com"
# Other option.
# OSS_BUCKET_DOMAIN="https://ocrtoc-public.oss-accelerate.aliyuncs.com"

echo "Downloading ocrtoc_materials..."
mkdir -p ocrtoc_materials
cd ocrtoc_materials
wget ${OSS_BUCKET_DOMAIN}/ocrtoc_materials/models.tar.gz
tar -zxvf models.tar.gz
rm -rf models.tar.gz
wget ${OSS_BUCKET_DOMAIN}/ocrtoc_materials/scenes.tar.gz
tar -zxvf scenes.tar.gz
rm -rf scenes.tar.gz
wget ${OSS_BUCKET_DOMAIN}/ocrtoc_materials/objects.csv

echo "Downloading sapien files..."
cd ..
wget ${OSS_BUCKET_DOMAIN}/sapien.tar.gz
tar -zxvf sapien.tar.gz
rm -rf sapien.tar.gz

echo "Build image."
docker build -t you_docker_image:tag_name .

echo "Done."
