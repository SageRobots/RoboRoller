labels = ["ROLLER_CAP_1V0", 
	"MTR_BRACK_X_1V0", 
	"ROLLER_BRACK_1V0", 
	"LC_BRACK_1V0", 
	"LC_BRACK_A_1V0",
	"LEAD_NUT_BRACK_X_1V0",
	"PCB_BRACK_1V0",
	"LC_BRACK_B_1V0",
	"LC_PCB_BRACK_1V0",
	"MTR_BRACK_Z_1V0",
	"LEAD_NUT_BRACK_Z_1V0",
	"CARRIAGE_PLATE_Z_1V0",
	"CARRIER_BRACK_A_1V0",
	"CARRIER_BRACK_B_1V0",
	"CARRIAGE_PLATE_X_1V0",
	"MAGNET_COUPLER_1V0",
	"ENC_BRACK_X_1V0",
	"HOME_BRACK_Z_1V0",
	"CARRIER_BRACK_C_1V0",
	"CARRIER_BRACK_D_1V0",
	"ENC_BRACK_Z_1V0",
	"END_CAP_1V0"]

for label in labels:
	path = "C:/Projects/MuscleRoller/STL/" + label + ".stl"

	__objs__=[]
	__objs__.append(FreeCAD.getDocument("MUSCLE_ROLLER_1V0").getObjectsByLabel(label)[0])
	import Mesh
	Mesh.export(__objs__, path)

	del __objs__