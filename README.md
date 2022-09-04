# PointCloudSequenceViewer

Viewer for multiple point cloud.  

# Environment

- macOS Monterey (ver. 12.5)  
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

# TODO

- Display pcd_file name of currently shown point cloud on top-left of viewer window.
- Automatically and Intermittently switches the displayed point cloud to next one, which can be set by args/keydown-on-window.
- Function to load multiple array of 3d bboxes/labels, in which an element (list of 3d bboxes/labels) is corresponding to single pcd_file.
- Show these 3d bboxes/labels of currently shown point cloud.
- Update cmake so as to be able to build on ubuntu.

<!-- # 気になるところ

- SequenceViewerにPointCloudを持たせる必要がない? -> もしくは、cloudをアップデート+viewerの更新をするメンバ関数を追加する?
- callbackの型がvoid* -> まぁまぁどうしようもない、、スマートポインタを使うなど、、?
- SequenceViewer::current_pcd_idはprotectedにした方が良い？ -->