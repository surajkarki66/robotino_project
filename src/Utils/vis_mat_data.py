from scipy.io import loadmat

data = loadmat('../BuildMapFrom2DLidarScansUsingSLAMExample/wareHouse.mat')
print(data.keys())


scans = data['wareHouseScans']
print(type(scans))
print(scans.shape)

scan0 = scans[0, 0]
# See keys and structure
print(scan0)
print(type(scan0))
print(scan0['arr'])       # Likely to contain a nested struct or array
print(scan0['arr'].dtype) # See if 'Ranges' and 'Angles' are in here
