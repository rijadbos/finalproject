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
            printf("Key Pressed: %c",key);
        }

        key=keypad_input();

        if(key != 0)
        {   
            if(key_position<4) //Limits to only 4 numbers
            {
                input_buffer[key_position] = key; //The position in the array is replaced with the number we pressed
                key_position++;

                for(int i=0;i<4;i++)
                {
                    printf("%c", input_buffer[i]);
                }
            }
            printf("Key Pressed: %c\n", key);
            key = 0;
        }

        if(key_position == 4 && password[0] == input_buffer[0] && password[1] == input_buffer[1] && password[2] == input_buffer[2] && password[3] == input_buffer[3])
        {
            correct=true;
            key_position=0;
        }
        else if(key_position==4 && password[0]!=input_buffer[0] && password[1]!=input_buffer[1] && password[2]!=input_buffer[2] && password[3]!=input_buffer[3])
        {
            correct=false;
            key_position=0;
        }
        if (correct==true)
        {
            printf("Ding");
        }
        else if (correct==false)
        {
            printf("Dong");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
