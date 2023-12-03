# motion-by-cnn

This is the sample ros node for using CNN with robot motion.  

## Dependeicies
Please refer `pyproject.xml` for detail.  

- tensorflow
- OpenCV

And this ros node uses [rye](https://rye-up.com/) for package management.  
And below package is the example of rye with ROS2.  

https://github.com/koichirokato/rye_sample_pkg


## Usage

### For image collection

```
ros2 run motion_by_cnn save_images.py
```

### For robot motion with CNN
You have to put `my-model.h5`, `labels.txt` of CNN network.  

```
ros2 run motion_by_cnn motion_by_cnn.py
```

## License
MIT License