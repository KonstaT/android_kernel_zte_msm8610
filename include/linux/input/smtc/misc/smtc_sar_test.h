
#ifndef _SMTC_SAR_TEST_H_
#define _SMTC_SAR_TEST_H_

#include "smtc_sar_test_platform_data.h"
#include <linux/types.h>
#include <linux/input.h>
struct _smtc_sar_test {
    u8 prox_body;

    u16 keycode;
    struct input_dev *input;

} typedef smtc_sar_test_t, *psmtc_sar_test_t;

// **************************************************************************
/*! \brief Clear out data
 *  \param this - pointer containing variables used for processing the test
 */
static inline void smtc_sar_clear(psmtc_sar_test_t this)
{
    if (this) {
        this->prox_body = -1;
    }
}

/*! \brief Remove any initialized data. Right now it is just the input
 *  \param this pointer containing variables used for processing the test
 */
static inline void smtc_sar_remove(psmtc_sar_test_t this)
{
    if (this) {
        if (this->input) {
            input_free_device(this->input);
            input_unregister_device(this->input);
            this->input = 0;
        }
    }
}

#define MAX_PROX_VAL    65535
#define INPUT_FUZZ	    32
#define INPUT_FLAT	    32

/*! \brief Initialize variables
 *  \description Call clear function and then initialize settings to their
 *  default values.
 *  \param this pointer containing variables used for processing the test
 *  \param pdata pointer containing platform specific values for computing
 *  the SAR test.
 */
static inline void smtc_sar_init(psmtc_sar_test_t this, psmtc_sar_test_platform_data_t pdata,
                                 struct device *parent, int bustype)
{
    if (this) {
        smtc_sar_clear(this);
        this->keycode = pdata->keycode;
        this->input = input_allocate_device();
        smtc_sar_clear(this);
        if (this->input) {
            set_bit(EV_ABS, this->input->evbit);
            set_bit(ABS_DISTANCE, this->input->mscbit);

			this->input->name = "attiny44a";
			//Should be "Semtech prox", avoid to make exist project change, use this name. zhaoyang196673
	
            this->input->id.bustype = bustype;
            this->input->dev.parent = parent;
            input_set_abs_params(this->input, ABS_DISTANCE, 0, MAX_PROX_VAL, INPUT_FUZZ, INPUT_FLAT);
            if (input_register_device(this->input))
                smtc_sar_remove(this);
        }
    }
}
/*! \brief Update input for SAR status
 *  \description this will perform the function to perform the event for
 *  the SAR status.
 *  \param this - pointer containing variables used for processing the test
 *  \param value - status on whether body is detected
 *  \return 1 - in body proximity, 0 - else
 */
#define DEV1_INT_VAL    1<<7
static inline unsigned char smtc_sar_state(psmtc_sar_test_t this, u8 value)
{
    if (this) {
        if (value) {
            if (this->prox_body!=1) {
                this->prox_body = 1;
                if (this->input) {

#ifdef RUNTIME_REG_PRINT					
                    dev_info( this->input->dev.parent,
                              "\t\t Reporting Proximity of Body\n");
#endif					
						
                    input_report_abs(this->input,ABS_DISTANCE,DEV1_INT_VAL);
                    input_sync(this->input);
                }
            }
            return 1;
        } else {
            if (this->prox_body!=0) {
                this->prox_body = 0;
                if (this->input) {

#ifdef RUNTIME_REG_PRINT					
                    dev_info( this->input->dev.parent,
                              "\t\t Reporting No Body Proximity\n");
#endif					
						
                    input_report_abs(this->input,ABS_DISTANCE,0);
                    input_sync(this->input);
                }
            }
            return 0;
        }
    }
    return 0; /* no memory was created, return 0 */
}
// **************************************************************************
#endif // _SMTC_SAR_TEST_H_
