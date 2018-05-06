/*
* CMD.ino
*
* Created: 11/15/2015 9:50:07 AM
*  Author: QuocTuanIT
*/

void init_command()
{
  Serial.println("======INIT CMD!!======");
	command.addCommand("on",LED_on);
	command.addCommand("off",LED_off);
  command.addCommand("unlock",unlockSystem);
	command.addCommand("rst",soft_reset);
	command.addCommand("pass",changePass);
 
	command.addDefaultHandler(unrecognized);
}
void unrecognized()
{
	Serial.println("WRONG CMD!!");
}
void LED_on()
{
	Serial.println("LED on");
	digitalWrite(LED_PIN,HIGH);
}
void LED_off()
{
	Serial.println("LED off");
	digitalWrite(LED_PIN,LOW);
}
void unlockSystem()
{
  Serial.print("Pass word:");
  char *arg = command.next();
  String passTYPE(arg);
  if (passTYPE != NULL)
  {
    Serial.println(passTYPE);
    Serial.println(globalPassWord);
    if(passTYPE == globalPassWord)
    {
      digitalWrite(LED_PIN,HIGH);
      Serial.println("pass correct");
    }
    else
    {
      digitalWrite(LED_PIN,LOW);
      Serial.println("pass incorrect");
    }
  }
  else Serial.println("??? param");  
}
void soft_reset()
{
		wdt_enable(WDTO_15MS);
		for(;;)
		{ }
}
void changePass()
{
	Serial.println("Set pass");
	char *arg = command.next();// chuoi sau "pass"
  char *arg1 = command.next();// chuoi sau "pass"
	if (arg != NULL )
	{
		Serial.println(arg);
    Serial.println(arg1);
    String oldPass(arg);
    String newPass(arg1);
    if(oldPass = globalPassWord)
    {
        Serial.println("Saving pass to eeprom...");
        EepromUtil::eeprom_write_string(100, newPass.c_str());
        globalPassWord = newPass;
    }
	}
	else Serial.println("??? param");
}

