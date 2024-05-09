//© 2023 Regents of the University of Minnesota. All rights reserved.

#include <HaarProg.h>

HaarProg::HaarProg(uint8_t talonPort_, uint8_t sensorPort_, uint8_t version)
{
	//Only update values if they are in range, otherwise stick with default values
	if(talonPort_ > 0) talonPort = talonPort_ - 1;
	else talonPort = 255; //Reset to null default if not in range
	if(sensorPort_ > 0) sensorPort = sensorPort_ - 1;
	else sensorPort = 255; //Reset to null default if not in range 
	sensorInterface = BusType::I2C; 
}

String HaarProg::begin(time_t time, bool &criticalFault, bool &fault)
{
	// Serial.println("HaarProg - BEGIN"); //DEBUG!
	//  //DEBUG!
	// if(gasSensor.begin(Wire, true) == false) {
	// 	Serial.println("\tSCD30 Init Fail"); //DEBUG!
	// 	throwError(SCD30_INIT_FAIL | talonPortErrorCode | sensorPortErrorCode); //Error subtype = I2C error
	// } 
	
	// Wire.beginTransmission(0x76);
	// int error = Wire.endTransmission();
	// if(error != 0) {
	// 	Serial.println("\tDPS368 Init Fail"); //DEBUG!
	// 	throwError(DPS368_INIT_ERROR | (error << 12) | talonPortErrorCode); //Error subtype = I2C error
	// }
	
	// Wire.beginTransmission(0x44);
	// int errorB = Wire.endTransmission();
	// ret = pres.measureTempOnce(temperature, oversampling);
	// Serial.print("INIT: ");
	// if(errorA == 0 || errorB == 0) Serial.println("PASS");
	// else {
	// 	Serial.print("ERR - ");
	// 	if(errorA != 0) {
	// 		Serial.print("A\t");
	// 		throwError(SHT3X_I2C_ERROR | (errorA << 12) | talonPortErrorCode); //Error subtype = I2C error
	// 	}
	// 	if(errorB != 0) Serial.print("B\t");
	// 	Serial.println("");
	// }
	return ""; //DEBUG!
}

String HaarProg::getMetadata()
{
	// Wire.beginTransmission(0x58); //Write to UUID range of EEPROM
	// Wire.write(0x98); //Point to start of UUID
	// int error = Wire.endTransmission();
	// // uint64_t uuid = 0;
	// String uuid = "";

	// if(error != 0) throwError(EEPROM_I2C_ERROR | error);
	// else {
	// 	uint8_t val = 0;
	// 	Wire.requestFrom(0x58, 8); //EEPROM address
	// 	for(int i = 0; i < 8; i++) {
	// 		val = Wire.read();//FIX! Wait for result??
	// 		// uuid = uuid | (val << (8 - i)); //Concatonate into full UUID
	// 		uuid = uuid + String(val, HEX); //Print out each hex byte
	// 		// Serial.print(Val, HEX); //Print each hex byte from left to right
	// 		// if(i < 7) Serial.print('-'); //Print formatting chracter, don't print on last pass
	// 		if(i < 7) uuid = uuid + "-"; //Print formatting chracter, don't print on last pass
	// 	}
	// }

	String metadata = "\"HaarProg\":{";
	// if(error == 0) metadata = metadata + "\"SN\":\"" + uuid + "\","; //Append UUID only if read correctly, skip otherwise 
	metadata = metadata + "\"Hardware\":\"v" + String(version >> 4, HEX) + "." + String(version & 0x0F, HEX) + "\","; //Report version as modded BCD
	metadata = metadata + "\"Firmware\":\"" + FIRMWARE_VERSION + "\","; //Static firmware version 
	metadata = metadata + "\"Pos\":[" + getTalonPortString() + "," + getSensorPortString() + "]"; //Concatonate position 
	metadata = metadata + "}"; //CLOSE  
	return metadata; 
	// return ""; //DEBUG!
}

String HaarProg::getData(time_t time)
{
	// float temperatureDPS368;
	// float pressure;
	// uint8_t oversampling = 7;
	// int16_t ret;
	
	String output = "\"HaarProg\":{"; //OPEN JSON BLOB
	// String dps368Data = "\"DPS368\":{\"Temperature\":"; //Open dps368 substring
	// String sht3xData = "\"SHT31\":{\"Temperature\":"; //Open SHT31 substring //FIX! How to deal with SHT31 vs SHT35?? Do we deal with it at all

	//lets the Dps368 perform a Single temperature measurement with the last (or standard) configuration
	//The result will be written to the paramerter temperature
	//ret = Dps368PressureSensor.measureTempOnce(temperature);
	//the commented line below does exactly the same as the one above, but you can also config the precision
	//oversampling can be a value from 0 to 7
	//the Dps 368 will perform 2^oversampling internal temperature measurements and combine them to one result with higher precision
	//measurements with higher precision take more time, consult datasheet for more information
	// Wire.beginTransmission(0x77);
	// int errorA = Wire.endTransmission();
	// Wire.beginTransmission(0x76);
	// int errorB = Wire.endTransmission();
	if(getSensorPort() != 0) { //If sensor detected
		bool dummy1;
		bool dummy2;
		time_t localTime = millis();
		while((millis() - wakeTime) < 5000 && (millis() - localTime) < 30000); //Wait for it to be 5 seconds since startup to have legit value, catch with a 30 second override 
		begin(0, dummy1, dummy2); //DEBUG!
		// ret = presSensor.measureTempOnce(temperatureDPS368, oversampling); //Measure temp
		localTime = millis();
		// while(gasSensor.dataAvailable() == false && (millis() - localTime) < 30000) { //Wait up to 30 seconds for new data //DEBUG!
		// 	delay(1s); 
		// }
		// if(gasSensor.dataAvailable() == false) {
		// 	throwError(SENSOR_TIMEOUT | talonPortErrorCode | sensorPortErrorCode | 0x200); //OR with new data timeout error
		// 	// Serial.println("CO2 Timeout!"); //DEBUG!
		// }
		uint8_t status = 1; //Use to check status of measurment 
		uint8_t attemptCount = 0; //Keep track of how many times a measure has been tried
		do {
			status = updateMeasurements(true); //Sync new readings
			attemptCount++;
		} while(status != 0 && attemptCount < attemptCountMax);
		// bool status = true; //DEBUG!
		if(status == 0) {
			// float co2 = gasSensor.getCO2();
			float temp = getTemperature();
			float pres = getPressure();
			float rh = getHumidity();
			output = output + "\"Pressure\":" + String(pres) + ",\"Temperature\":" + String(temp) + ",\"Humidity\":" + String(rh); //Concatonate read values
		}
		else { //If no error in read
			output = output + "\"Pressure\":null,\"Temperature\":null,\"Humidity\":null"; //Concatonate null string
			// throwError(SCD30_I2C_FAIL | talonPortErrorCode | sensorPortErrorCode); //Throw error on connection
			// dps368Data = dps368Data + String(temperatureDPS368,2) + ","; //Append temp with 2 decimal points since resolution is 0.01°C, add comma
			// dps368Data = dps368Data + "Pressure" + String()
		}
		if(attemptCount > 0) throwError(REPEATED_READ_ATTEMPT | talonPortErrorCode | sensorPortErrorCode);
	}
	else {
		throwError(FIND_FAIL); //Report failure to find
		output = output + "\"Pressure\":null,\"Temperature\":null,\"Humidity\":null"; //Concatonate null string
	}
	

	// Serial.print("TEMP: ");
	// if(errorA == 0 || errorB == 0) Serial.println(temperature); 
	// else {
	// 	Serial.print("ERR - ");
	// 	if(errorA != 0) Serial.print("A\t");
	// 	if(errorB != 0) Serial.print("B\t");
	// 	Serial.println("");
	// }
	
	output = output + ",";
	output = output + "\"Pos\":[" + getTalonPortString() + "," + getSensorPortString() + "]"; //Concatonate position 
	output = output + "}"; //CLOSE JSON BLOB
	Serial.println(output); //DEBUG!
	return output;
}

bool HaarProg::isPresent() 
{ //FIX!
	Wire.beginTransmission(ADR_base);
	int errorA = Wire.endTransmission();

	Wire.beginTransmission(ADR_alt);
	int errorB = Wire.endTransmission();
	// Serial.print("HaarProg TEST: "); //DEBUG!
	// Serial.print(error);
	if(errorA == 0 || errorB == 0) {
		ADR = (errorA == 0) ? ADR_base : ADR_alt; //Set the internal ADR value dependent on which device version is found
		return true; //If one of the versions is found, return true. Otherwise false
	}
	else return false;
}

// void HaarProg::setTalonPort(uint8_t port)
// {
// 	// if(port_ > numPorts || port_ == 0) throwError(PORT_RANGE_ERROR | portErrorCode); //If commanded value is out of range, throw error 
// 	if(port > 4 || port == 0) throwError(TALON_PORT_RANGE_ERROR | talonPortErrorCode | sensorPortErrorCode); //If commanded value is out of range, throw error //FIX! How to deal with magic number? This is the number of ports on KESTREL, how do we know that??
// 	else { //If in range, update the port values
// 		talonPort = port - 1; //Set global port value in index counting
// 		talonPortErrorCode = (talonPort + 1) << 4; //Set port error code in rational counting 
// 	}
// }

// void HaarProg::setSensorPort(uint8_t port)
// {
// 	// if(port_ > numPorts || port_ == 0) throwError(PORT_RANGE_ERROR | portErrorCode); //If commanded value is out of range, throw error 
// 	if(port > 4 || port == 0) throwError(SENSOR_PORT_RANGE_ERROR | talonPortErrorCode | sensorPortErrorCode); //If commanded value is out of range, throw error //FIX! How to deal with magic number? This is the number of ports on KESTREL, how do we know that??
// 	else { //If in range, update the port values
// 		sensorPort = port - 1; //Set global port value in index counting
// 		sensorPortErrorCode = (sensorPort + 1); //Set port error code in rational counting 
// 	}
// }

// String HaarProg::getSensorPortString()
// {
// 	if(sensorPort >= 0 && sensorPort < 255) return String(sensorPort + 1); //If sensor port has been set //FIX max value
// 	else return "null";
// }

// String HaarProg::getTalonPortString()
// {
// 	if(talonPort >= 0 && talonPort < 255) return String(talonPort + 1); //If sensor port has been set //FIX max value
// 	else return "null";
// }

// int HaarProg::throwError(uint32_t error)
// {
// 	errors[(numErrors++) % MAX_NUM_ERRORS] = error; //Write error to the specified location in the error array
// 	if(numErrors > MAX_NUM_ERRORS) errorOverwrite = true; //Set flag if looping over previous errors 
// 	return numErrors;
// }

String HaarProg::getErrors()
{
	// if(numErrors > length && numErrors < MAX_NUM_ERRORS) { //Not overwritten, but array provided still too small
	// 	for(int i = 0; i < length; i++) { //Write as many as we can back
	// 		errorOutput[i] = error[i];
	// 	}
	// 	return -1; //Throw error for insufficnet array length
	// }
	// if(numErrors < length && numErrors < MAX_NUM_ERRORS) { //Not overwritten, provided array of good size (DESIRED)
	// 	for(int i = 0; i < numErrors; i++) { //Write all back into array 
	// 		errorOutput[i] = error[i];
	// 	}
	// 	return 0; //Return success indication
	// }
	String output = "\"HaarProg\":{"; // OPEN JSON BLOB
	output = output + "\"CODES\":["; //Open codes pair

	for(int i = 0; i < min(MAX_NUM_ERRORS, numErrors); i++) { //Interate over used element of array without exceeding bounds
		output = output + "\"0x" + String(errors[i], HEX) + "\","; //Add each error code
		errors[i] = 0; //Clear errors as they are read
	}
	if(output.substring(output.length() - 1).equals(",")) {
		output = output.substring(0, output.length() - 1); //Trim trailing ','
	}
	output = output + "],"; //close codes pair
	output =  output + "\"OW\":"; //Open state pair
	if(numErrors > MAX_NUM_ERRORS) output = output + "1,"; //If overwritten, indicate the overwrite is true
	else output = output + "0,"; //Otherwise set it as clear
	output = output + "\"NUM\":" + String(numErrors) + ","; //Append number of errors
	output = output + "\"Pos\":[" + getTalonPortString() + "," + getSensorPortString() + "]"; //Concatonate position 
	output = output + "}"; //CLOSE JSON BLOB
	numErrors = 0; //Clear error count
	return output;

	// return -1; //Return fault if unknown cause 
}

float HaarProg::getPressure(bool update) //Get pressure in mBar
{
	if(update) updateMeasurements(); //Only call for updated value if requested
	uint32_t val = 0; //Val for getting/calculating pressure value
	uint32_t Temp = 0; //DEBUG

	for(int i = 0; i < 3; i++) {
		Wire.beginTransmission(ADR);
		Wire.write(PRES_REG + i);
		Wire.endTransmission();
		Wire.requestFrom(ADR, 1);
		Temp = Wire.read(); //DEBUG!
		val = (Temp << 8*i) | val; //DEBUG!
	}
	dataRequested = false; //Clear flag on data retreval
  	return (val / 4096.0);
}

float HaarProg::getHumidity(bool update)  //Return humidity in % (realtive)
{
	if(update) updateMeasurements(); //Only call for updated value if requested
	float val = 0; //Val for getting/calculating RH value
	val = (uint16_t(getWord(RH_REG)));
	val = (100.0*val)/65535.0;  //Convert to RH
	dataRequested = false; //Clear flag on data retreval
	return val;
}

float HaarProg::getTemperature(SensorInternal device, bool update)  //Return temp in C
{
	if(update) updateMeasurements(); //Only call for updated value if requested
	float val = 0; //Val for getting/calculating temp value
	if(device == Pres_Sense) {
		val = getWord(TEMP_PRES);
		val = val/100.0; //Convert to C
	}

	else if (device == RH_Sense) {
		val = getWord(TEMP_RH);
		val = ((val*175.0)/65535.0) - 45; //Convert to C
	}
	dataRequested = false; // Clear flag on data retreval
	return val;
}

// FIX! Allow for read of status register in order to not overwrite state bits
uint8_t HaarProg::updateMeasurements(bool block)
{
	dataRequested = true; // Set flag
	Wire.beginTransmission(ADR);
	Wire.write(0x00);
	Wire.write(0x01); // Trigger conversion
	uint8_t error = Wire.endTransmission(); // Return I2C status
	// Only block if triggered
	if(block) {
		// Get timeout value
		unsigned long timeout = millis();
		// Wait for new data to be returned
		while(!newData() && (millis() - timeout < timeoutGlobal)) {
			delay(1);
		}
		return error;
	}
	else return error;
}

bool HaarProg::newData()  // Checks for updated data
{
	unsigned long timeout = millis(); // Get timeout value
	Wire.beginTransmission(ADR);
	Wire.write(0x00);
	Wire.endTransmission();
	Wire.requestFrom(ADR, 1);
	// Wait for value to be returned //FIX! add timeout/remove
	while(Wire.available() < 1 && (millis() - timeout < timeoutGlobal)) {
		delay(1);
	}
	uint8_t val = Wire.read();  //DEBUG!
	bool state = false;
	// bool state = ~(val & 0x01);
	if(val & 0x01 == 1) state = false;  //FIX! Make cleaner
	else state = true;
	// Serial.println(state); //DEBUG!
	// Return inverse of bit 0, true when bit has been cleared,
	// false when waiting for new conversion
	return (state);
}

int16_t HaarProg::getWord(uint8_t Reg)  //Returns word, read from Reg position
{
	uint16_t val = 0; //Val to be read from device
	Wire.beginTransmission(ADR);
	Wire.write(Reg);
	Wire.endTransmission();

	Wire.requestFrom(ADR, 1);  // Request word
	//while(Wire.available() < 2); //Wait //FIX! Add timeout
	val = Wire.read();
	// Serial.println(val, HEX);

  Wire.beginTransmission(ADR);
  Wire.write(Reg + 1);
  Wire.endTransmission();
  Wire.requestFrom(ADR, 1);  //Request word
	val = val | (Wire.read() << 8);  //Concatonate 16 bits
	//val = Wire.read() | (val << 8);  //Concatonate 16 bits //DEBUG!
	return val;
}