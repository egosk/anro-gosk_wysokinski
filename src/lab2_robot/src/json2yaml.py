import math
import yaml
import json

print 'dziala'

with open('dh_data.json') as jsonFile:
	dh = json.load(jsonFile)

print 'dziala2'

with open('yaml_data.yaml', 'w') as yamlFile:
    print 'stworzono plik'
    for i in dh.keys():
        row = dh[i]
        
        yamlFile.write(i + ':\n')
        yamlFile.write('   '+ 'length:'+ str(row[1])+'\n')
        yamlFile.write('   '+ 'xyz:'+ '0.0 '+ str(row[i])+str(row[+'\n')
