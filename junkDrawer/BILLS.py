
# Function to sum all values in a dictionary.
def sumValuesInADictionary(DICT):
	ret = 0
	for index, key in enumerate(DICT.keys()):
		ret += DICT[f"{key}"]
	return ret

# Dictionary of bills and their costs.
BILLS = {
	"GROCERIES": 200,
	"GASOLINE": 200,
	"RENT": 1050,
	"TUITION": 1100,
	"CAR_INSURANCE": 115,
	"DISCOVER_STUDENT_LOAN": 113,
	"CAR_BILL": 200, # Goes away after November.
	"POWER_AND_WATER": 135,
	"WIFI": 80,
	"AMAZON_PRIME": 16,
	"PHONE": 130, # Hopefully goes away after December.
	"CHEGG": 16.50,
	"APPLE_CLOUD_STORAGE": 1,
	"CAR_WASH": 20,
	"GYM_MEMBERSHIP": 25
}

# Constants.
TOTAL_PER_MONTH = sumValuesInADictionary(DICT=BILLS)
PAYCHECK = 1735

# Current bank balance.
CURRENT_BALANCE = 2397.55

# Create september bills.
SEPTEMBER_BILLS = BILLS

# Running list of bills that have been paid.
SEPTEMBER_BILLS.pop("RENT")

# Calculate bills left in September
SEPTEMBER_BILLS_LEFT = sumValuesInADictionary(SEPTEMBER_BILLS)

# Estimate money left at the end of this month.
AT_END_OF_SEPTEMBER = CURRENT_BALANCE - SEPTEMBER_BILLS_LEFT + (PAYCHECK * 2)
print("AT END OF SEPTEMBER", AT_END_OF_SEPTEMBER)

# Correct for paid off computer loan from work.
AT_END_OF_SEPTEMBER += 58
PAYCHECK += 58 # Computer loan paid off.

# The rest of the year.
AT_END_OF_OCTOBER = AT_END_OF_SEPTEMBER - TOTAL_PER_MONTH + (PAYCHECK * 2)
print("AT END OF OCTOBER ", AT_END_OF_OCTOBER)

AT_END_OF_NOVEMBER = AT_END_OF_OCTOBER - TOTAL_PER_MONTH + (PAYCHECK * 2)
print("AT END OF NOVEMBER ", AT_END_OF_NOVEMBER)

TUITION_REIMBURSEMENT = 3508
AT_END_OF_DECEMBER = AT_END_OF_NOVEMBER - TOTAL_PER_MONTH + BILLS["TUITION"] + (PAYCHECK * 2) + TUITION_REIMBURSEMENT + BILLS["CAR_BILL"] # Car paid off.
print("AT END OF DECEMBER ", AT_END_OF_DECEMBER)