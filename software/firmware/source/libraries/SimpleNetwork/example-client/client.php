<?php
$fp = fsockopen("127.0.0.1", 11999, $errno, $errstr, 30);
if (!$fp) {
    echo "$errstr ($errno)<br />\n";
} else {
    fwrite($fp, "Hello =)");
    while (!feof($fp)) {
        echo fgets($fp, 128);
    }
    fclose($fp);
} 
