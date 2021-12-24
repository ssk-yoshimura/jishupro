# jishupro

## eusモデルの作り方
SolidWorksからstlを出力するときは、解像度を下げて、単位をmにする

https://github.com/jsk-ros-pkg/jsk_model_tools

例
```bash
roscd jishupro/models
rosrun collada_urdf_jsk_patch urdf_to_collada sample_model.urdf sample_model.dae
rosrun euscollada collada2eus sample_model.dae sample_model.l
```
