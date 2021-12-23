let myip = '192.168.4.1';

(function() {
    'use strict';

    let files_tree_json = [];

    const xhr = new XMLHttpRequest();
    xhr.responseType = "json";

    xhr.addEventListener("load", e => {
        if (xhr.status === 200) {
            filestree(xhr.response);
        }
    });

    xhr.addEventListener("error", e => {
        // alert("Cant connect, use template");
        filestree(files_tree_json);
    });

    xhr.onreadystatechange = function() { // Call a function when the state changes.
        if (this.readyState === XMLHttpRequest.DONE && this.status === 404) {
            // Request finished. Do processing here.
            filestree(files_tree_json);
        }
    }

    let current_path = '';

    function getfilestree(path)
    {
        current_path = path;
        console.log(path);
        xhr.open("GET", path + '/', true);
        xhr.send();    
    }


    function filestree(json){
        let tree = document.getElementById("filestree");
        let _path = document.getElementById("full_path");
        _path.innerText = current_path;
        tree.innerHTML = "<th>Name</th><th>Type</th><th>Size</th><th>Delete</th>";
    
        console.log(json);
        let n = 0;
        json.forEach(element => {
            let tr = document.createElement("tr");
            tr.className = 'filestree';
            let a = document.createElement("a");
            if (element.dir === 1) {
                a.href = '#';
                a.target = "_self";
                a.link = element.path;
                a.onclick = function (event) {
                    event.preventDefault();
                    getfilestree(element.path);
                };
            } else{
                a.href = element.path;
                a.target = "_blank";
            }
            a.text = element.name;
            let td1 = document.createElement("td");
            td1.appendChild(a);
            let td2 = document.createElement("td");
            td2.innerHTML = element.dir?"directory":"file";
            let td3 = document.createElement("td");
            td3.innerHTML = element.size;
            td1.className = 'filestree';
            td2.className = 'filestree1';
            td3.className = 'filestree1';

            tr.appendChild(td1);
            tr.appendChild(td2);
            tr.appendChild(td3);
            

            let btnDel = document.createElement("button");
            btnDel.path = element.path;
            btnDel.className = 'button black';
            if (n > 0) {                
                btnDel.innerText = "delete";
                btnDel.addEventListener('click', () => {
                    const http = new XMLHttpRequest();
                    var params = 'file=' + element.path;
                    var url = '/delete';
                    http.open("POST", url, true);
                    // Send the proper header information along with the request
                    http.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
                
                    http.onreadystatechange = function() {//Call a function when the state changes.
                        if(http.readyState == 4 && http.status == 201) {
                            console.log('element deleted', http.responseText);
                            getfilestree(current_path);
                        } else if (http.readyState == 4 && http.status == 409) {
                            console.log('cant delete', element.path);
                        }
                    }
                    http.send(params);
                });
            } else {
                btnDel.innerText = "mkdir";
                btnDel.addEventListener('click', () => {
                    let dirname = prompt("New directory name");
                    if(dirname == null || dirname == "") return;
                    const http = new XMLHttpRequest();
                    let sub;
                    if (element.path.substring(element.path.length - 3, element.path.length) === '/..') {
                        sub = element.path.substring(0, element.path.length - 2);
                    } else {
                        sub = element.path + '/';
                    }
                    var params = 'dirname=' + sub + dirname;
                    console.log(params);
                    var url = '/mkdir';
                    http.open("POST", url, true);
                    // Send the proper header information along with the request
                    http.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
                
                    http.onreadystatechange = function() {//Call a function when the state changes.
                        if(http.readyState == 4 && http.status == 201) {
                            console.log('directory created', http.responseText);
                            getfilestree(current_path);
                        } else if (http.readyState == 4 && http.status == 409) {
                            alert('directory already exist');
                        }
                    }
                    http.send(params);
                });
            }

            let td4 = document.createElement("td");
            td4.appendChild(btnDel);
            td4.className = 'filestree1';
            tr.appendChild(td4);

            tree.appendChild(tr);
            n++;
        });
    }
    
    function postwifiap(path, ssid, pass, ap, auth)
    {
        const http = new XMLHttpRequest();
    
        console.log(path);
        var params = 'ssid=' + ssid + '&pass=' + pass + '&ap=' + ap + '&auth=' + auth;
        var url = '/wifi';
        http.open("POST", url, true);
        // Send the proper header information along with the request
        http.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
    
        http.onreadystatechange = function() {//Call a function when the state changes.
            if(http.readyState == 4 && http.status == 200) {
                console.log('credentials set', http.responseText);
            }
        }
        http.send(params);
    }
    
    function postsettings(freq, bw, sf, cr)
    {
        const http = new XMLHttpRequest();
    
        var params = 'freq=' + freq + '&bw=' + bw + '&sf=' + sf + '&cr=' + cr;
        var url = '/settings';
        http.open("POST", url, true);
        // Send the proper header information along with the request
        http.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
    
        http.onreadystatechange = function() {//Call a function when the state changes.
            if(http.readyState == 4 && http.status == 200) {
                console.log('settings set', http.responseText);
            }
        }
        http.send(params);
    }
    
    document.addEventListener('DOMContentLoaded', event => {

        getfilestree('/files');

        let stabtn = document.getElementById("sta_btn");
        let apbtn = document.getElementById("ap_btn");
        let enable_ap = document.getElementById("en_ap");
        let enable_sta = document.getElementById("en_sta");
        let settingsSend = document.getElementById("settingsSend");

        // Get the modal
        let modalstats = document.getElementById("myModal");
        let modalwifi = document.getElementById("myModal1");
        let modaltree = document.getElementById("myModal2");
        let spinner = document.getElementById("spinner");
        let settingsForm = document.getElementById("settingsForm");
        
        // Get the button that opens the modal
        let settingsBtn = document.getElementById("settingsBtn");
        let btnwifi = document.getElementById("wifi");
        let btntree = document.getElementById("tree");
        let btnformat = document.getElementById("format");
        let btnmkdir = document.getElementById("mkdir");

        // Settings fields
        let FREQ = document.querySelector("#freq1");
        let BW = document.querySelector("#bw1");
        let SF = document.querySelector("#sf1");
        let CR = document.querySelector("#cr1");

        try {
            FREQ.value = init_freq;
            BW.value = init_bw;
            SF.value = init_sf;
            CR.value = init_cr;           
        } catch (error) {}
/*
        btnformat.onclick = function() {
            if(confirm("Do you want to format SD crad?\nIt will takes about 5 minutes!!!")){
                spinner.display = 'block';
                const http = new XMLHttpRequest();
                var url = '/format';
                http.open("POST", url, true);
            
                http.onreadystatechange = function() {//Call a function when the state changes.
                    if(http.readyState == 4 && http.status == 200) {
                        console.log('sd card formatted', http.responseText);
                        spinner.display = 'none';
                    }
                }
                http.send();
            }
        }
*/
        let _btnmkdir = function() {
        // btnmkdir.onclick = function() {
            let dirname = prompt("New directory name");
            if(dirname == null || dirname == "") return;
            const http = new XMLHttpRequest();
            var params = 'dirname=' + dirname;
            var url = '/mkdir';
            http.open("POST", url, true);
            // Send the proper header information along with the request
            http.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
        
            http.onreadystatechange = function() {//Call a function when the state changes.
                if(http.readyState == 4 && http.status == 201) {
                    console.log('directory created', http.responseText);
                    getfilestree(current_path);
                } else if (http.readyState == 4 && http.status == 409) {
                    console.log('directory already exist', http.responseText);
                    alert('directory already exist');
                }
            }
            http.send(params);
        }

        btnwifi.onclick = function() {
            modalwifi.style.display = "block";
        }
        btntree.onclick = function() {
            modaltree.style.display = "block";
        }
        enable_ap.onclick = function(ev) {
            console.log('AP', ev);
            let content = document.getElementById("ap_content");
            if(ev.target.checked) content.style.display = "block";
            else content.style.display = "none";
        }
        enable_sta.onclick = function(ev) {
            let content = document.getElementById("sta_content");
            if(ev.target.checked) content.style.display = "block";
            else content.style.display = "none";
        }
        // Get the <span> element that closes the modal
        // let span1 = document.getElementsByClassName("close1")[0];
        let span2 = document.getElementById("close2");
        let span3 = document.getElementById("close3");
        let span4 = document.getElementById("close4");
        
        // When the user clicks on <span> (x), close the modal
        // span1.onclick = function() {
        //     modalstats.style.display = "none";
        // }
        span2.onclick = function() {
            modalwifi.style.display = "none";
        }
        // span3.onclick = function() {
        //     modaltree.style.display = "none";
        // }
        
        // When the user clicks anywhere outside of the modal, close it
        window.onclick = function(event) {
            if (event.target == modalstats) {
                modalstats.style.display = "none";
            }
            if (event.target == modalwifi) {
                modalwifi.style.display = "none";
            }
            if (event.target == modaltree) {
                modaltree.style.display = "none";
            }
            if (event.target == settingsForm) {
                settingsForm.style.display = "none";
            }
        }

        stabtn.addEventListener('click', function() {
            const ssid = document.querySelector("#sta_ssid").value;
            const pass = document.querySelector("#sta_pass").value;
            postwifiap('', ssid, pass, 0, 0);
        });

        apbtn.addEventListener('click', function() {
            const ssid = document.querySelector("#ap_ssid").value;
            const pass = document.querySelector("#ap_pass").value;
            var auth = document.getElementById("auth");
            postwifiap('', ssid, pass, 1, auth.value);
        });

    });

})();