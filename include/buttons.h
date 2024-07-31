#pragma once

#include "defines.h"
#include "globals.h"
#include "display.h"
#include "config.h"

void setupButtons();
void menuMainKP1(void);

void menuConfigKP1_next(void);
void menuConfigKP2_select(void);
void menuConfigLP2_exit(void);

void menuConfigAlterKP1_select(void);
void menuConfigAlterKP2_edit(void);
void menuConfigAlterLP1_save(void);
void menuConfigAlterLP2_cancel(void);

void menuConfigAlterValKP1_incr(void);
void menuConfigAlterValKP2_dec(void);
void menuConfigAlterValLP1_save(void);
void menuConfigAlterValLP2_cancel(void);