#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import requests

# #del all vehicle
# for i in range(0,24):
#     url = "http://robotx.cowbon.info/vehicles/"+str(i)
#     print(url)
#
#     payload = {}
#     headers= {}
#
#     response = requests.request("DELETE", url, headers=headers, data = payload)
#
#     print(response.text.encode('utf8'))
#
#
#
# #post my vehicle
# url = "http://robotx.cowbon.info/vehicles"
#
# payload = 'vname=yellow'
# headers = {
#   'Content-Type': 'application/x-www-form-urlencoded'
# }
#
# response = requests.request("POST", url, headers=headers, data = payload)
#
# print(response.text.encode('utf8'))


#get veh
import requests

url = "http://robotx.cowbon.info/vehicles/24"

payload = {}
headers= {}

response = requests.request("GET", url, headers=headers, data = payload)

print(response.text.encode('utf8'))
