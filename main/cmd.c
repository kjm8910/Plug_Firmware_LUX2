
#include "mosa.h"


extern int mosa_cmd_data_get(int argc, char **argv);
extern int mosa_cmd_data_set(int argc, char **argv);

int mosa_cmd_init()
{
    APP_command_reg("mos-get", "mg", mosa_cmd_data_get, "get mos data");
    APP_command_reg("mos-set", "ms", mosa_cmd_data_set, "set mos data");

    return (1);
}


int mosa_cmd_data_get(int argc, char **argv)
{
    printf("get mos data\n");

    return (1);
}


int mosa_cmd_data_set(int argc, char **argv)
{
    int i;

    printf("set mos data\n");

    for (i = 0; i < argc; i++) {
        printf("%d = %s\n", i, argv[i]);
    }

    return (1);
}

/* end of file */
