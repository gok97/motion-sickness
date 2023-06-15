# Motion-Sickness

#### Installation
The rosbags package requires at least Python version >= 3.8.2

Create a python virtual enviroment and activate it 
```
mkdir .venvs && cd .venvs
python3 -m venv ms_venv
source ms_venv/bin/activate
```

Install requirements
```
pip install requirements.txt
```

#### Setup
Save ros2 bag files in the bags directory.

#### Running the code
```
python3 process_bags.py
```