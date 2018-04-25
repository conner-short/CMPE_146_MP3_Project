#include <stddef.h>
#include <string.h>

#include "command_handler.hpp"
#include "VS1053.hpp"

VS1053* mp3CmdDec = NULL;

CMD_HANDLER_FUNC(mp3Handler) {
    if(mp3CmdDec == NULL)
    {
        output.putline("Error: MP3 decoder not initialized");
        return false;
    }

    char* op = NULL;
    char* arg = NULL;

    cmdParams.tokenize(" ", 2, &op, &arg);

    if(op == NULL) {
        return false;
    }

    if(strncmp(op, "play", strlen("play")) == 0)
    {
        if(arg != NULL)
        {
            if(!mp3CmdDec->play(arg))
            {
                output.printf("Error: Error playing MP3 file %s\n", arg);
            }
        }
        else
        {
            mp3CmdDec->setPlayType(VS1053::PLAY);
            mp3CmdDec->resume();
        }
    }
    else if(strncmp(op, "pause", strlen("pause")) == 0)
    {
        mp3CmdDec->pause();
    }
    else if(strncmp(op, "stop", strlen("stop")) == 0)
    {
        mp3CmdDec->stop();
    }
    else if(strncmp(op, "ff", strlen("ff")) == 0)
    {
        mp3CmdDec->setPlayType(VS1053::FF);
    }
    else
    {
        return false;
    }

    return true;
}
