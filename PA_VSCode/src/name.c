
#include <usb_names.h>

#define PRODUCT_NAME                                                            \
  {                                                                       \
    'T', 'e', 'e', 'n', 's', 'y', ' ', '4', '.', 'O' \
  }
#define PRODUCT_NAME_LEN 10

#define MANUFACTURER_NAME                                                            \
  {                                                                       \
    'P', 'J', 'R', 'C' \
  }
#define MANUFACTURER_NAME_LEN 4

#define SERIAL_NUMBER                                            \
  {                                                        \
    ',','P', 'o', 'r', 't'\
  }
#define SERIAL_NUMBER_LEN 5


struct usb_string_descriptor_struct usb_string_manufacturer_name = {
  2 + MANUFACTURER_NAME_LEN * 2,
  3,
  MANUFACTURER_NAME};

struct usb_string_descriptor_struct usb_string_product_name = {
  2 + PRODUCT_NAME_LEN * 2,
  3,
  PRODUCT_NAME};

struct usb_string_descriptor_struct usb_string_serial_number = {
  2 + SERIAL_NUMBER_LEN * 2,
  3,
  SERIAL_NUMBER};
