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
cd .. && pip install -r requirements.txt
```

#### Setup
Save ros2 bag files in the bags directory.

#### Running the code
Parse ros2 bags into csv files
```
python3 process_bags.py
```

Straight way route - plot acceleration and jerk
```
python3 plot_acc.py
python3 plot_jerk.py
```

Plot routes on MCity map
```
python3 plot_route.py
```