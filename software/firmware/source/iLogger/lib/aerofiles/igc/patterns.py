from sys import implementation

if implementation.name == 'micropython':
  from ure import compile

if implementation.name == 'circuitpython':
  from re import compile

DATE = compile('^([0-3][0-9])([0-1][0-9])([0-9][0-9])$')
TIME = compile('^([0-2][0-9])([0-5][0-9])([0-5][0-9])$')
DATETIME = compile('^([0-3][0-9])([0-1][0-9])([0-9][0-9])'
                      '([0-2][0-9])([0-5][0-9])([0-5][0-9])$')

THREE_LETTER_CODE = compile('^[A-Z0-9][A-Z0-9][A-Z0-9]$')
MANUFACTURER_CODE = THREE_LETTER_CODE
LOGGER_ID = THREE_LETTER_CODE
EXTENSION_CODE = THREE_LETTER_CODE
