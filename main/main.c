/*====HEADERS=====*/
#include "projectheader.h"

/*====C FILES=====*/
#include "i2c_main.c"
#include "keypad.c"

/*====FUNCTIONS=====*/
/*====VARIABLES=====*/
char key;
int key_position = 0;

char password[4] = {'8','0','0','8'};
char input_buffer[4];

bool correct = false;
bool asked = false;


/*====MAIN====*/
void app_main(void)
{
    i2c_configuration();
    keypad_configuration();

    printf("System Starting...");
    while(1)
    {
        if(asked==false) //Prints only once at the start of the while loop
        {
            printf("Enter Key Code: \n");
            asked=true;
        }

        key=keypad_input();

        if(key != 0)
        {
            printf("Key Pressed: %c",key);
            input_buffer[key_position] = key; //The position in the array is replaced with the number we pressed
            
            if(key_position<4) //Limits to only 4 numbers
            {
                key_position++;
            }
        }

        if(key_position == 3 && password[0] == input_buffer[0] && password[1] == input_buffer[1] && password[2] == input_buffer[2] && password[3] == input_buffer[3])
        {
            correct=true;
        }
        else
        {
            correct=false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
