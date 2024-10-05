# generating docs
You can generate docs by going into the `mycobot_client_2/mycobot_client_2` folder and then running the below:
```
python -m pydoc ik_pybullet>docs.txt
```

And then cleaning up the output by deleting the ros stuff that gets inherited. You can also take screenshots of the pretty html docs:
```
python -m pydoc ik_pybullet -p 1234
```