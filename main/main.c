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
bool output = false;


/*====MAIN====*/
void app_main(void)
{
    i2c_configuration();
    keypad_configuration();

    while(1)
    {
        if(asked==false) //Prints only once at the start of the while loop
        {
            printf("System Starting...\n");
            printf("Enter Key Code: \n");
            asked=true;
        }

        key=keypad_input();

        if(key != 0)
        {   
            if(key_position<4) //Limits to only 4 numbers
            {
                input_buffer[key_position]=key; //buffers the number currently pressed to an array position
                key_position++; //updates the position of the password number to the next element for the next turn
            }
            printf("Key Pressed: %c\n", key);
            key = 0;
        }

        if(key_position == 4)
        {
            correct=true;
            for (int i=0;i<4;i++)
            {
                if(input_buffer[i]!=password[i])
                {
                    correct=false;
                }
            }
            if(correct && !output)
            {
                output=true;
                printf("Password:%c%c%c%c is correct\n",input_buffer[0],input_buffer[1],input_buffer[2],input_buffer[3]);
            }
            else if(!correct && !output)
            {
                output=true;
                printf("Password:%c%c%c%c is incorrect\n",input_buffer[0],input_buffer[1],input_buffer[2],input_buffer[3]);
            }

            key_position=0;
            output=false;
            asked=false;

        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
