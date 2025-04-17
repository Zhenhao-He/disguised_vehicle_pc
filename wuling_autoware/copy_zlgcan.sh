#!/bin/bash

# 自动获取 Conda 环境路径
CONDA_ENV_PATH="${CONDA_PREFIX}"

# 检查 CONDA_PREFIX 是否存在
if [ -z "$CONDA_ENV_PATH" ]; then
  echo "Error: No Conda environment is activated."
  exit 1
fi

# 源文件夹路径
SOURCE_PATH="./src/vehicleinterface/USBCAN-8E-U"

# 目标文件夹路径
INCLUDE_PATH="${CONDA_ENV_PATH}/include"
LIB_PATH="${CONDA_ENV_PATH}/lib"

# 复制 zlgcan 文件夹到 include 目录
if [ -d "${SOURCE_PATH}/zlgcan" ]; then
    echo "Copying zlgcan to ${INCLUDE_PATH}..."
    cp -r "${SOURCE_PATH}/zlgcan" "${INCLUDE_PATH}/"
else
    echo "Error: ${SOURCE_PATH}/zlgcan does not exist."
    exit 1
fi

# 复制 libusbcan-8e.so 到 lib 目录
if [ -f "${SOURCE_PATH}/libusbcan-8e.so" ]; then
    echo "Copying libusbcan-8e.so to ${LIB_PATH}..."
    cp "${SOURCE_PATH}/libusbcan-8e.so" "${LIB_PATH}/"
else
    echo "Error: ${SOURCE_PATH}/libusbcan-8e.so does not exist."
    exit 1
fi


