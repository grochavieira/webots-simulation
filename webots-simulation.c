#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <webots/supervisor.h>

#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>

#define TIME_STEP 64

#define DISTANCE_SENSORS_NUMBER 8
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static double distance_sensors_values[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

#define LEDS_NUMBER 10
static WbDeviceTag leds[LEDS_NUMBER];
static bool leds_values[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};

static WbDeviceTag left_motor, right_motor;

static int get_time_step()
{
    static int time_step = -1;
    if (time_step == -1)
        time_step = (int)wb_robot_get_basic_time_step();
    return time_step;
}

static void init_devices()
{
    int i;
    for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++)
    {
        distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
        wb_distance_sensor_enable(distance_sensors[i], get_time_step());
    }

    for (i = 0; i < LEDS_NUMBER; i++)
        leds[i] = wb_robot_get_device(leds_names[i]);
}

static void reset_actuator_values()
{
    int i;
    for (i = 0; i < LEDS_NUMBER; i++)
        leds_values[i] = false;
}

static void set_actuators()
{
    int i;
    for (i = 0; i < LEDS_NUMBER; i++)
        wb_led_set(leds[i], leds_values[i]);
}

static void blink_leds()
{
    static int counter = 0;
    leds_values[counter] = true;
}

static void get_sensor_input()
{
    int i;
    for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++)
    {
        distance_sensors_values[i] = wb_distance_sensor_get_value(distance_sensors[i]);

        // scale the data in order to have a value between 0.0 and 1.0
        // 1.0 representing something to avoid, 0.0 representing nothing to avoid
        distance_sensors_values[i] /= 4096;
    }
}

static int my_exit(void)
{
    wb_robot_cleanup();
    return 0;
}

int main(int argc, char **argv)
{
    /* necessary to initialize webots stuff */
    wb_robot_init();
    init_devices();
    printf("Iniciou o controller...\n");

    int caixasLeves = 0;

    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);

    WbNodeRef robot_node = wb_supervisor_node_get_from_def("ePuck"); //captura o supervisor
    if (robot_node == NULL)
    {
        fprintf(stderr, "No DEF MY_ROBOT node found in the current world file\n");
        exit(1);
    }

    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation"); //identifica o campo de posição
    const double *posicao;                                                            //variáel que vai receber a posição do robo
    double posicaoAntesX = 0.0;
    double posicaoAntesZ = 0.0;
    double posicaoDepoisX = 0.0;
    double posicaoDepoisZ = 0.0;

    while (true)
    {

        double lm_value = 0.0;
        double rm_value = 0.0;

        double maiorPS1 = 0.0;
        double maiorPS6 = 0.0;

        if (distance_sensors_values[0] > 0.2 || distance_sensors_values[7] > 0.2)
        {
            if (distance_sensors_values[0] > distance_sensors_values[7])
            {
                printf("PS0\n");
                lm_value = 3.0;
                rm_value = -3.0;
            }
            else
            {
                printf("PS7\n");
                lm_value = -3.0;
                rm_value = 3.0;
            }
        }
        else
        {
            lm_value = 3.0;
            rm_value = 3.0;
        }

        posicao = wb_supervisor_field_get_sf_vec3f(trans_field);

        posicaoAntesX = posicao[0];
        posicaoAntesZ = posicao[2];

        set_actuators();
        double t = wb_robot_get_time();
        while (wb_robot_get_time() - t < 1.3)
        {
            posicao = wb_supervisor_field_get_sf_vec3f(trans_field);
            printf("Posicao do robo: x= %f   y= %f z= %f\n", posicao[0], posicao[1], posicao[2]);
            get_sensor_input();

            printf("valor do ps0: %f\n", distance_sensors_values[0]);
            printf("valor do ps1: %f\n", distance_sensors_values[1]);
            printf("valor do ps6: %f\n", distance_sensors_values[6]);
            printf("valor do ps7: %f\n", distance_sensors_values[7]);

            if (distance_sensors_values[6] > maiorPS6)
            {
                maiorPS6 = distance_sensors_values[6];
            }

            if (distance_sensors_values[1] > maiorPS1)
            {
                maiorPS1 = distance_sensors_values[1];
            }

            wb_motor_set_velocity(left_motor, lm_value);
            wb_motor_set_velocity(right_motor, rm_value);

            if (wb_robot_step(TIME_STEP) == -1)
                return my_exit();
        }

        posicaoDepoisX = posicao[0];
        posicaoDepoisZ = posicao[2];

        printf("MAIORPS6: %f", maiorPS6);
        printf("MAIORPS1: %f", maiorPS1);

        if (maiorPS1 > 0.1 || maiorPS6 > 0.1)
        {
            printf("\n*************ENCOSTOU EM ALGO*************\n");
            double difPosicaoX = fabs(fabs(posicaoAntesX) - fabs(posicaoDepoisX));
            double difPosicaoZ = fabs(fabs(posicaoAntesZ) - fabs(posicaoDepoisZ));
            if ((difPosicaoX <= 0.04) || (difPosicaoZ <= 0.04) || (posicao[0] <= -0.36 || posicao[2] <= -0.36 || posicao[0] >= 0.36 || posicao[2] >= 0.36))
            {
                printf("\n++++++++++PAREDE OU CAIXA PESADA++++++++++\n");
            }
            else
            {
                printf("\n==========LEDS==========\n");
                blink_leds();
                set_actuators();
            }
        }
        else
        {
            reset_actuator_values();
        }

        printf("ANTES --- Posicao do robo: x= %f  z= %f\n", posicaoAntesX, posicaoAntesZ);
        printf("DEPOIS --- Posicao do robo: x= %f  z= %f\n", posicaoDepoisX, posicaoDepoisZ);
    }

    /* Enter your cleanup code here */

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}
