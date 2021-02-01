import json
import requests

for test_time in range(0,50):
    import requests

    url = "http://robotx.cowbon.info/anchorwaypoints/"+str(test_time)
    payload = {}
    headers= {}
    response = requests.request("GET", url, headers=headers, data = payload)
    tmp = response.json()
    if not 'aid' in tmp.keys() : continue
    print(tmp)

# import requests
#
# url = "http://robotx.cowbon.info/anchorwaypoints/34"
#
# payload = 'mid=1&globalx=12.345678901&globaly=9.0&uwbid=1'
# headers = {
#   'Content-Type': 'application/x-www-form-urlencoded'
# }
#
# response = requests.request("PUT", url, headers=headers, data = payload)
#
# print(response.text.encode('utf8'))
