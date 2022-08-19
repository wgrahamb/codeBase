GROCERIES = 200
GASOLINE = 200 # Gonna pay this per month off of shell credit card until paid off.
RENT = 1050
TUITION = 1100
CAR_INSURANCE_ACTUAL = 148.38
CAR_INSURANCE_PLACEHOLDER = 35 # Amazon prime card monthly payment. Accidentally paid of the rest of my premium on this card.
STUDENT_LOANS = 113
CAR_BILL = 200
POWER_AND_WATER = 147.07
WIFI = 80
PRIME = 16.19
PHONE = 130
CHEGG = 16.52
APPLE_CLOUD_STORAGE = 0.99
TIDAL_CAR_WASH = 20

TOTAL_PER_MONTH = GROCERIES + GASOLINE + RENT + TUITION + CAR_INSURANCE_PLACEHOLDER + STUDENT_LOANS + CAR_BILL + POWER_AND_WATER + WIFI + PRIME + PHONE + CHEGG + APPLE_CLOUD_STORAGE + TIDAL_CAR_WASH

PAYCHECK = 1726.99

CURRENT_BALANCE = 2526.95
GROCERY_MONEY_SPENT_IN_JULY = 44.69 + 23.56 + 17.10 + 60.12
if GROCERY_MONEY_SPENT_IN_JULY > GROCERIES:
	print(f"GROCERY LIMIT EXCEEDED.")
BILLS_LEFT_IN_AUGUST = TOTAL_PER_MONTH - RENT - GROCERY_MONEY_SPENT_IN_JULY - STUDENT_LOANS - CAR_BILL - GASOLINE - CAR_INSURANCE_PLACEHOLDER - PHONE
PAYING_BACK_DAD = 875
TUITION_REIMBURSEMENT = 1992
AT_END_OF_AUGUST = CURRENT_BALANCE - BILLS_LEFT_IN_AUGUST + TUITION_REIMBURSEMENT - PAYING_BACK_DAD
print("AT END OF AUGUST ", AT_END_OF_AUGUST)

# I'm praying for a raise.

AT_END_OF_SEPTEMBER = AT_END_OF_AUGUST - TOTAL_PER_MONTH + (PAYCHECK * 3)
print("AT END OF SEPTEMBER", AT_END_OF_SEPTEMBER)

AT_END_OF_SEPTEMBER += 58
PAYCHECK += 58 # Computer loan paid off.

AT_END_OF_OCTOBER = AT_END_OF_SEPTEMBER - TOTAL_PER_MONTH + (CAR_INSURANCE_PLACEHOLDER - CAR_INSURANCE_ACTUAL) + (PAYCHECK * 2)
print("AT END OF OCTOBER ", AT_END_OF_OCTOBER)

AT_END_OF_NOVEMBER = AT_END_OF_OCTOBER - TOTAL_PER_MONTH + (CAR_INSURANCE_PLACEHOLDER - CAR_INSURANCE_ACTUAL) + (PAYCHECK * 2)
print("AT END OF NOVEMBER ", AT_END_OF_NOVEMBER)

TUITION_REIMBURSEMENT = 3508
AT_END_OF_DECEMBER = AT_END_OF_NOVEMBER - TOTAL_PER_MONTH + (CAR_INSURANCE_PLACEHOLDER - CAR_INSURANCE_ACTUAL) + TUITION + (PAYCHECK * 2) + TUITION_REIMBURSEMENT + CAR_BILL # Car paid off.
print("AT END OF DECEMBER ", AT_END_OF_DECEMBER)

# Pay off the sum total of Shell gas card and Amazon credit card at the end of the year.