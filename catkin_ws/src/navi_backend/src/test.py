import json
import requests

url = "http://robotx.cowbon.info/anchorwaypoints/274"
payload = 'mid=1&globalx=-9&globaly=0.0&uwbid=1'
headers = {
  'Content-Type': 'application/x-www-form-urlencoded'
}

response = requests.request("PUT", url, headers=headers, data = payload)

print(response.text.encode('utf8'))
