/*====HEADERS=====*/
#include "projectheader.h"

/*====VARIABLES=====*/

char keypad_keys[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

int row_pins[4] = {4,5,6,7};
int col_pins[4] = {8,9,10,11};

/*====FUNCTION DECLERATIONS====*/
void keypad_configuration();
char keypad_input();

/*====FUNCTION DEFINITIONS====*/

    /* Function Name - keypad_configuration
    * Description   - 
    *     Configure the stupid keypad gpios
    * Parameters    - None
    * Return type   - void
    */
void keypad_configuration()
{
    for(int i=0;i<4;i++) //ROW PINS CONFIGURATION
    {
        gpio_set_direction(row_pins[i],GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(row_pins[i],GPIO_PULLUP_ONLY);
    }
      for(int j=0;j<4;j++) //COLLUMN PINS CONFIGURATION
    {
        gpio_set_direction(col_pins[j],GPIO_MODE_INPUT);
        gpio_set_level(col_pins[j], 1);
    }
}

    /* Function Name - keypad_input
    * Description   - 
    *     Reads the keypad that was pressed using the collumn and row pins we use on our ESP 
    * Parameters    - None
    * Return type   - char
    */
char keypad_input()
{
    for(int i=0;i<=4;i++)
    {
        gpio_set_level(row_pins[i],0);

        for(int j=0;j<4;j++)
        {
            if(gpio_get_level(col_pins[j]) == 0)
            {
                vTaskDelay(150/portTICK_PERIOD_MS);
                gpio_set_level(row_pins[i], 1);
                return keypad_keys[i][j];
            }
        }

        gpio_set_level(row_pins[i],1);
    }


    return 0;
}

