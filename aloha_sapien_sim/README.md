# aloha sapien doc

本文档用于aloha在sapien环境中的仿真




## Quick start

- [File Directory Description](#file-directory-description)
- [Installation](#installation)
  - [Dependencies](#dependencies)
  - [Build from Source](#build-from-source)
- [Start](#start)

- [Author](#author)

---
### File Directory Description

```
aloha_sapien_sim
|── meshes
|── srdf
|── urdf
├── README.md
├── test.py
```


### env init

Python versions:

* Python 3.8

Operating systems:

* Linux: Ubuntu 18.04+, Centos 7+

---

首先你需要配置一下sapien的运行环境
```bash
conda create -n aloha_sapien python=3.8
conda activate aloha_sapien
```

由于box2_link2.dae文件太大，在上传到github前压缩了这个文件，我们需要对这个文件进行解压：
```bash
# 在meshes文件夹下打开终端
unzip box2_Link.dae.zip
```

#### **Dependencies**



* Sapien: 3.0.0b1
```bash
pip install sapien==3.0.0b1
```

* mplib: 0.1.1
```bash
pip install mplib==0.1.1
```

由于提供的模型文件较为精细，有些模型文件不能通过sapien自动convex，但是是否convex并不影响仿真的效果，因此需要对mplib的planner类init做一个小改动，在pip install完mplib后找到本conda环境下的mplib/planner.py,对71行做更改：将convex=True修改为convex=False 。

```
ps: 通常这个文件的路径为anaconda3/envs/aloha_sapien/lib/python3.8/site-packages/mplib/planner.py
```



### Start

```bash
cd aloha_sapien_sim
python test.py
```

### author
 *ShijiaPeng*

