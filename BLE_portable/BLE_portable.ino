 // BlueTooth mobile - reciver BT-Rec
 // Taken from advertising examples. 

/*  This example scans for advertising devices (peripherals) in range,
 *  looking for a specific UUID in the advertising packet. When this
 *  UUID is found, it will display an alert, sorting devices detected
 *  in range by their RSSI value, which is an approximate indicator of
 *  proximity (though highly dependent on environmental obstacles, etc.).
 *  
 *  A simple 'bubble sort' algorithm is used, along with a simple
 *  set of decisions about whether and where to insert new records
 *  in the record list.
 *  
 *  This example is intended to be used with multiple peripherals
 *  devices running the *_peripheral.ino version of this application.
 *  
 *  VERBOSE_OUTPUT
 *  --------------
 *  Verbose advertising packet analysis can be enabled for
 *  advertising packet contents to help debug issues with the 
 *  advertising payloads, with fully parsed data displayed in the 
 *  Serial1 Monitor.
 *  
 *  ARRAY_SIZE
 *  ----------
 *  The numbers of peripherals tracked and sorted can be set via the
 *  ARRAY_SIZE macro. Must be at least 2.
 *  
 *  TIMEOUT_MS
 *  ----------
 *  This value determines the number of milliseconds before a tracked
 *  peripheral has it's last sample 'invalidated', such as when a device
 *  is tracked and then goes out of range.
 *  
 */

#include <string.h>
#include <bluefruit.h>
#include <SPI.h>

#define ARRAY_SIZE     (4)    // The number of RSSI values to store and compare
#define TIMEOUT_MS     (2500) // Number of milliseconds before a record is invalidated in the list


#if (ARRAY_SIZE <= 1)
  #error "ARRAY_SIZE must be at least 2"
#endif


// Custom UUID used to differentiate this device.
// Use any online UUID generator to generate a valid UUID.
// Note that the byte order is reversed ... CUSTOM_UUID
// below corresponds to the follow value:
// df67ff1a-718f-11e7-8cf7-a6006ad3dba0  
const uint8_t CUSTOM_UUID[] =
{
    0xA0, 0xDB, 0xD3, 0x6A, 0x00, 0xA6, 0xF7, 0x8C,
    0xE7, 0x11, 0x8F, 0x71, 0x1A, 0xFF, 0x67, 0xDF
};

BLEUuid uuid = BLEUuid(CUSTOM_UUID);



/* This struct is used to track detected nodes */
typedef struct node_record_s
{
  uint8_t  addr[6];    // Six byte device address
  int8_t   rssi;       // RSSI value
  uint32_t timestamp;  // Timestamp for invalidation purposes
  int8_t   reserved;   // Padding for word alignment
} node_record_t;

node_record_t records[ARRAY_SIZE];

void setup()
{
  Serial1.begin(9600);
  Serial.begin(9600);
  //while ( !Serial1 ) delay(10);   // for nrf52840 with native usb

  Serial.println("Bluefruit52 Central Proximity Example");
  Serial.println("-------------------------------------\n");




  /* Clear the results list */
  memset(records, 0, sizeof(records));
  for (uint8_t i = 0; i<ARRAY_SIZE; i++)
  {
    // Set all RSSI values to lowest value for comparison purposes,
    // since 0 would be higher than any valid RSSI value
    records[i].rssi = -128;
  }

  /* Enable both peripheral and central modes */
  if ( !Bluefruit.begin(1, 1) )
  {
    Serial.println("Unable to init Bluefruit");
    while(1)
    {
      digitalToggle(LED_RED);
      delay(100);
    }
  }
  else
  {
    Serial.println("Bluefruit initialized (central mode)");
  }
  
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  /* Set the device name */
  Bluefruit.setName("Bluefruit52");

  /* Set the LED interval for blinky pattern on BLUE LED */
  Bluefruit.setConnLedInterval(250);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Filter out packet with a min rssi
   * - Interval = 100 ms, window = 50 ms
   * - Use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.filterRssi(-80);            // Only invoke callback for devices with RSSI >= -80 dBm
  Bluefruit.Scanner.filterUuid(uuid);           // Only invoke callback if the target UUID was found
  //Bluefruit.Scanner.filterMSD(0xFFFF);          // Only invoke callback when MSD is present with the specified Company ID
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);        // Request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
  Serial.println("Scanning ...");
}

/* This callback handler is fired every time a valid advertising packet is detected */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  node_record_t record;
  
  /* Prepare the record to try to insert it into the existing record list */
  memcpy(record.addr, report->peer_addr.addr, 6); /* Copy the 6-byte device ADDR */
  record.rssi = report->rssi;                     /* Copy the RSSI value */
  record.timestamp = millis();                    /* Set the timestamp (approximate) */

  /* Attempt to insert the record into the list */
  if (insertRecord(&record) == 1)                 /* Returns 1 if the list was updated */
  {
    //printRecordList();                            /* The list was updated, print the new values */
    //Serial1.println("");
    
}

  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}



/* Prints the current record list to the Serial1 Monitor */
void printRecordList(void)
{

    //Serial1.printf("[%i] ", i);
    Serial1.printBuffer(records[0].addr, 6, ':');
    Serial1.printf(" %i \n", records[0].rssi);
    //Serial1.printf(" %i (%u ms)\n", records[0].rssi, records[0].timestamp);

}

/* This function performs a simple bubble sort on the records array */
/* It's slow, but relatively easy to understand */
/* Sorts based on RSSI values, where the strongest signal appears highest in the list */
void bubbleSort(void)
{
  int inner, outer;
  node_record_t temp;

  for(outer=0; outer<ARRAY_SIZE-1; outer++)
  {
    for(inner=outer+1; inner<ARRAY_SIZE; inner++)
    {
      if(records[outer].rssi < records[inner].rssi)
      {
        memcpy((void *)&temp, (void *)&records[outer], sizeof(node_record_t));           // temp=records[outer];
        memcpy((void *)&records[outer], (void *)&records[inner], sizeof(node_record_t)); // records[outer] = records[inner];
        memcpy((void *)&records[inner], (void *)&temp, sizeof(node_record_t));           // records[inner] = temp;
      }
    }
  }
}

/*  This function will check if any records in the list
 *  have expired and need to be invalidated, such as when
 *  a device goes out of range.
 *  
 *  Returns the number of invalidated records, or 0 if
 *  nothing was changed.
 */
int invalidateRecords(void)
{
  uint8_t i;
  int match = 0;

  /* Not enough time has elapsed to avoid an underflow error */
  if (millis() <= TIMEOUT_MS)
  {
    return 0;
  }

  /* Check if any records have expired */
  for (i=0; i<ARRAY_SIZE; i++)
  {
    if (records[i].timestamp) // Ignore zero"ed records
    {
      if (millis() - records[i].timestamp >= TIMEOUT_MS)
      {
        /* Record has expired, zero it out */
        memset(&records[i], 0, sizeof(node_record_t));
        records[i].rssi = -128;
        match++;
      }
    }
  }

  /* Resort the list if something was zero'ed out */
  if (match)
  {
    // Serial1.printf("Invalidated %i records!\n", match);
    bubbleSort();    
  }

  return match;
}

/* This function attempts to insert the record if it is larger than the smallest valid RSSI entry */
/* Returns 1 if a change was made, otherwise 0 */
int insertRecord(node_record_t *record)
{
  uint8_t i;
  
  /* Invalidate results older than n milliseconds */
  invalidateRecords();
  
  /*  Record Insertion Workflow:
   *  
   *            START
   *              |
   *             \ /
   *        +-------------+
   *  1.    | BUBBLE SORT |   // Put list in known state!
   *        +-------------+
   *              |
   *        _____\ /_____
   *       /    ENTRY    \    YES
   *  2. <  EXISTS W/THIS > ------------------+
   *       \   ADDRESS?  /                    |
   *         -----------                      |
   *              | NO                        |
   *              |                           |
   *       ______\ /______                    |
   *      /      IS       \   YES             |
   *  3. < THERE A ZERO'ED >------------------+
   *      \    RECORD?    /                   |
   *        -------------                     |
   *              | NO                        |
   *              |                           |
   *       ______\ /________                  |
   *     /     IS THE       \ YES             |
   *  4.<  RECORD'S RSSI >=  >----------------|
   *     \ THE LOWEST RSSI? /                 |
   *       ----------------                   |
   *              | NO                        |
   *              |                           |
   *             \ /                         \ /
   *      +---------------+           +----------------+
   *      | IGNORE RECORD |           | REPLACE RECORD |
   *      +---------------+           +----------------+
   *              |                           |
   *              |                          \ /
   *             \ /                  +----------------+
   *             EXIT  <------------- |   BUBBLE SORT  |
   *                                  +----------------+
   */  

  /* 1. Bubble Sort 
   *    This puts the lists in a known state where we can make
   *    certain assumptions about the last record in the array. */
  bubbleSort();

  /* 2. Check for a match on existing device address */
  /*    Replace it if a match is found, then sort */
  uint8_t match = 0;
  for (i=0; i<ARRAY_SIZE; i++)
  {
    if (memcmp(records[i].addr, record->addr, 6) == 0)
    {
      match = 1;
    }
    if (match)
    {
      memcpy(&records[i], record, sizeof(node_record_t));
      goto sort_then_exit;
    }
  }

  /* 3. Check for zero'ed records */
  /*    Insert if a zero record is found, then sort */
  for (i=0; i<ARRAY_SIZE; i++)
  {
    if (records[i].rssi == -128)
    {
      memcpy(&records[i], record, sizeof(node_record_t));
      goto sort_then_exit;
    }
  }

  /* 4. Check RSSI of the lowest record */
  /*    Replace if >=, then sort */
  if (records[ARRAY_SIZE-1].rssi <= record->rssi)
  {
      memcpy(&records[ARRAY_SIZE-1], record, sizeof(node_record_t));
      goto sort_then_exit;
  }

  /* Nothing to do ... RSSI is lower than the last value, exit and ignore */
  return 0;

sort_then_exit:
  /* Bubble sort */
  bubbleSort();
  return 1;
}

void loop() 
{
  /* Toggle red LED every second */
  digitalToggle(LED_RED);

  /* Invalidate old results once per second in addition
   * to the invalidation in the callback handler. */
  /* ToDo: Update to use a mutex or semaphore since this
   * can lead to list corruption as-is if the scann results
   * callback is fired in the middle of the invalidation
   * function. */
  delay(10000);
  invalidateRecords();
  if (Serial1.available())
  {
    /* The list was updated, print the new values */
    printRecordList();
    Serial1.println("");

  }
 
  
}
