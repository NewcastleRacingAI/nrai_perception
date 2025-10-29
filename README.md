# NRAI perception

Newcastle Racing AI module for perception

To start working on this project, clone this repository.

```bash
git clone --recurse-submodules https://github.com/NewcastleRacingAI/nrai_perception.git
cd nrai_perception
```

## Project structure

```bash
nrai_perception
├── resource
│   └── nrai_perception # File marking the ros package name
├── src
│   └── nrai_perception
│       ├── ros.py      # Ros node 
│       ├── ...         # Main code
│       └── __init__.py
├── test                # Tests
├── package.xml         # Ros2 package configuration
├── setup.cfg           # Ros2 python paths
└── setup.py            # Python configuration
```

- **To add python dependencies** add them to the `setup.py` file
- **To add ros2 depedencies** add them to the `package.xml` file
