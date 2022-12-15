# PointCloudSequenceViewer

Viewer for multiple point cloud.  

# Environment

- macOS > Monterey (ver. 12.5)  
- Homebrew installed  


# Installation

1. Install Qt, Boost, PCL  

```
$ brew install qt@5 boost pcl
```

2. Build  

    2-1. Edit `CmakeLists.txt`  

    > Make sure to match your environment such like versions of libraries you installed above instruction.  

    2-2. Compile source codes  
    
    ```
    $ cd PointCloudSequenceViewer
    $ make
    ```


# Usage

```
./cloud_viewer --pcd_path [path/to/.pcd_file|directory]
```

`--pcd_path` is supposed to be path to `.pcd` file or directory under which `.pcd` files exist (directly).  

Baisically, manipulation of popuped window follows [usage of PCLVisualizer](https://pcl.readthedocs.io/projects/tutorials/en/master/pcl_visualizer.html#compiling-and-running-the-program).  

We add extra KeyDownEvents below.  
- right-allow : switch currently shown point cloud to next one.  
- left-allow : switch currently shown point cloud to previous one.  
- c : save current camera pose.  
- i : save current window screenshot to [current_dir/screenshot_pcl_viewer.png].
- shift + click point : show coord of clicked point.


# TODO

- [x] Display general info  
    - [x] pcd_file name of currently shown point cloud on top-left of viewer window.  
- [] Automatically and intermittently switches the displayed point cloud to next one, which can be configured by args/keydown-on-window.  
    - [x] intermittently switching functionality with left/right keydown.  
    - [] automatic frame proceeding  
- [] mouse callback  
    - [x] get coord of licked point.  
    - [] display coord of point clicked.  
    - [] debug  
- [] Keyboard callback  
    - [x] save screenshots  
    = [] save screenshots to specified directory.
    - [] automatically save screenshots for all frames.  
- [x] save/load camera configuration  
    - [x] save function  
    - [x] save path option  
    - [x] load function  
    - [x] load option  
- [] Function to load multiple array of 3d bboxes/labels, in which an element (list of 3d bboxes/labels) is corresponding to single pcd_file.  
    - [x] load single frame annotation file(3d bboxes/labels).  
    - [x] options to load annotation files  
    - [x] Show these 3d bboxes/labels of currently shown point cloud.  
    - [x] show bbox text label.  
    - [x] switch annotation to next/back one, when frame changes.  
    - [] size, extent clipping for bbox.  
    - [] debug  
- [] cmake file  
    - [] Update cmake so as to be able to build on ubuntu.  
    - [] Optimizing build like -O3 option.  
- [] migration to Qt  

<!-- # 気になるところ

- SequenceViewerにPointCloudを持たせる必要がない? -> もしくは、cloudをアップデート+viewerの更新をするメンバ関数を追加する?
- callbackの型がvoid* -> まぁまぁどうしようもない、、スマートポインタを使うなど、、?
- SequenceViewer::current_pcd_idはprotectedにした方が良い？ -->