import ure

DATE = ure.compile('^([0-3][0-9])([0-1][0-9])([0-9][0-9])$')
TIME = ure.compile('^([0-2][0-9])([0-5][0-9])([0-5][0-9])$')
DATETIME = ure.compile('^([0-3][0-9])([0-1][0-9])([0-9][0-9])'
                      '([0-2][0-9])([0-5][0-9])([0-5][0-9])$')

THREE_LETTER_CODE = ure.compile('^[A-Z0-9][A-Z0-9][A-Z0-9]$')
MANUFACTURER_CODE = THREE_LETTER_CODE
LOGGER_ID = THREE_LETTER_CODE
EXTENSION_CODE = THREE_LETTER_CODE
