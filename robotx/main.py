#!/usr/bin/python
import json
import requests

vehicle = 'yellow'

temp = None
for test_time in range(0,999):
    url = "http://robotx.cowbon.info/vehicles/"+str(test_time)
    payload = {}
    headers= {}
    response = requests.request("GET", url, headers=headers, data = payload)
    tmp = response.json()
    if not 'vid' in tmp.keys() : continue

    if tmp['vname'] == vehicle :
        print(tmp['vname'])
        break
