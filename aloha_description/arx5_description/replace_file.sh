#!/bin/bash

# 获取当前目录的名称
current_dir=$(basename "$PWD")

# 输入需要修改的名称
read -p "Enter the name to be replaced (default: current directory name): " old_name
if [ -z "$old_name" ]; then
    old_name=$current_dir
fi

# 输入替换的名称
read -p "Enter the new name: " new_name
if [ -z "$new_name" ]; then
    echo "The replacement name cannot be empty!"
    exit 1
fi

# 转义正则表达式的特殊字符
old_name_escaped=$(printf '%s\n' "$old_name" | sed -e 's/[]\/$*.^|[]/\\&/g')
new_name_escaped=$(printf '%s\n' "$new_name" | sed -e 's/[&/\]/\\&/g')


# 显示将什么替换为什么
echo "Replacing '${old_name}' to '${new_name}'."

# 替换当前目录及子目录下所有文件内容
find . -type f | while read -r file; do
    # 计算替换的次数
    count=$(grep -oF "${old_name}" "$file" | wc -l)
    if [ "$count" -gt 0 ]; then
        # 执行替换
        sed -i "s/${old_name}/${new_name}/g" "$file"
        echo "Modified $file - Made $count replacements."
    fi
done

# 替换当前目录及子目录下的文件名
find . -depth -type f -name "*$old_name*" -execdir bash -c 'mv "$0" "${0//$1/$2}"' {} "$old_name" "$new_name" \;

echo "Replacement completed!"
