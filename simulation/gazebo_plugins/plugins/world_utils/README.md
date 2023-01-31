### World Utils

This utility allows models to be added and removed programatically during simulation execution.

#### Custom textures

In order to use custom textures, the recommended procedure is as follows:

Create a folder media in the root with:

```
cd ~/workspace/gazebo-utils &&
mkdir media
```

The textures have to respect the following file structure:

```
media/
  └╴materials/
    ├╴scripts/
    │ └╴scipt_name.material
    └╴textures/
      └╴img.png
```

The sdf model itself is not needed, but the contents of `script_name.material` should resemble (the filtering is optional):

```
material Material/Name
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture img.png
        filtering anistropic
        max_anisotropy 16
      }
    }
  }
}
```

The pattern generation tool is a good example of how to generate materials that follow these guidelines.