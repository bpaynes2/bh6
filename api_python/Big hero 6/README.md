Download Python 3.9.8 64-bit
https://www.python.org/downloads/release/python-398/
	Check box : Add to Path

Visual studio set up
-Clone repo
--Run this command from Desktop location: git clone https://github.com/bpaynes2/bh6.git
--Set python interpreter to the path that says 3.9.8 AND DOES NOT SAY WINDOWS STORE

--Kortex whl file
---Download from: https://artifactory.kinovaapps.com/ui/api/v1/download?repoKey=generic-public&path=kortex%2FAPI%2F2.3.0%2Fkortex_api-2.3.0.post34-py3-none-any.whl
---Travel to where whl file is downloaded
----Run this command: pip -m install .\kortex_api-2.3.0.post34-py3-none-any.whl

LibrealSense
-Download SDK 2.0 from: https://github.com/IntelRealSense/librealsense/releases/download/v2.49.0/Intel.RealSense.SDK-WIN10-2.49.0.3474.exe
--Download pyrealsense2 from: pip install pyrealsense2

BEFORE ADDING ANY CHANGES, 
-create a new branch: git checkout -b "insert name"    no quotes