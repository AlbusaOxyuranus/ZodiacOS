; Ïî÷àòêîâèé çàâàíòàæóâà÷ ÿäðà â àðõ³òåêòóð³ õ86
format Binary as "bin"
org 0x7C00 ; âêàçóºìî áàçîâèé àäðåñ ïðîãðàìè, â³ä ÿêîãî ïîò³ì â³äðàõîâóþòüñÿ àäðåñè ì³òîê, à çàîäíî ³ âèä³ëÿº ì³ñöå äëÿ ñòåêó
	jmp boot ; ïðîïóñêàºìî ñòðóêòóðè äàíèõ ³ ïåðåõîäèìî íà êîä çàâàíòàæóâà÷à - êîìàíäà çàéìàº 3 áàéòè ïàìÿò³
	
; Çàãîëîâîê ListFS - íàøî¿ ôàéëîâî¿ ñèñòåìè
align 4 ;  âèð³âíÿòè ñòóêòóðó ïî 4áàéòí³é ãðàíèö³.
fs_magic dd ?
fs_version dd ?
fs_flags dd ?
fs_base dq ?
fs_size dq ?
fs_map_base dq ?
fs_map_size dq ?
fs_first_file dq ?
fs_uid dq ?
fs_block_size dd ?
; Çàãîëîâîê ôàéëó
virtual at 0x800
f_info:
	f_name rb 256
	f_next dq ?
	f_prev dq ?
	f_parent dq ?
	f_flags dq ?
	f_data dq ?
	f_size dq ?
	f_ctime dq ?
	f_mtime dq ?
	f_atime dq ?
end virtual

; Äàí³ ïî÷àòêîâîãî çàâàíòàæóâà÷à - ïåðåâêðèâàºìî ñòàðòîâó êîìàíäó jmp
label sector_per_track word at $$ ; 2 áàéòè - çì³ííà sector_per_track ðîçì³ùóºòüñÿ ïî àäðåñó çàäàíîìó â îñòàííüîìó îïåðàòîð³ org
label head_count byte at $$ + 2 ; 1 áàéòè - base addr + 2
label disk_id byte at $$ + 3 ; 1 áàéò - base addr + 3 - òóò çáåðã³àòèìåòüñÿ íîìåð çàâàíòàæóâàëüíîãî äèñêó
reboot_msg db "Press any key...",13,10,0
boot_file_name db "boot.bin",0

; Ïðîöåäóðà âèâîäèòü íà åêðàí ðÿäîê ÿêèé ðîçì³ùåíèé ïî àäðåñ³ DS:SI
write_str:
	push si
	mov ah, 0x0E
	; 0eH ïèñàòü ñèìâîë íà àêòèâíóþ âèäåî ñòðàíèöó (ýìóëÿöèÿ òåëåòàéïà)
    ; âõîä:  AL = çàïèñûâàåìûé ñèìâîë (èñïîëüçóåò ñóùåñòâóþùèé àòðèáóò)
    ;        BL = öâåò ïåðåäíåãî ïëàíà (äëÿ ãðàôè÷åñêèõ ðåæèìîâ)
 @@:
	lodsb
	test al, al ; ïåðåâ³ðÿºìî ÷è ïîòî÷íèé ñèìâîë íå 0
	jz @f ; ÿêùî òàê òî âèõîäèìî ç öèêëó
	int 0x10 ; â³äåî ñåðâ³ñ

	jmp @b
 @@:
	pop si
	ret
	
; Âèêëèêàºòüñÿ ïðè âèíèêíåíí³ êðèòè÷íî¿ ïîìèëêè
error:
	pop si ; âèòÿãóºìî àäðåñ ïîâåðíåííÿ ç ôóíêö³¿ (òîáòî íàñòóïíèé ðÿäîê ï³ñëÿ âèêëèêó ôóíêö³¿) äå ðîçì³ùóºòüñÿ ïîâ³äîìëåííÿ ïðî ïîìèëêó
	call write_str
	
; Ïåðåçàâàíòàæåííÿ
reboot:
	mov si, reboot_msg ; âèâîäèìî ïîâ³äîìëåííÿ "Press any key..."
	call write_str
	
	; ÷åêàºìî íàòèñíåííÿ êëàâ³ø³
	xor ah, ah
	int 0x16 ; ñåðâ³ñ ââåäåííÿ âèâåäåííÿ êëàâ³àòóðè
	; 00H ÷èòàòü (îæèäàòü) ñëåäóþùóþ íàæàòóþ êëàâèøó
    ;  âûõîä: AL = ASCII ñèìâîë (åñëè AL=0, AH ñîäåðæèò ðàñøèðåííûé êîä ASCII )
    ;         AH = ñêàíêîä  èëè ðàñøèðåííûé êîä ASCII
	
	jmp 0xFFFF:0
	
; Çàâàíòàæåííÿ ñåêòîðà çà àäðåñîþ DX:AX â áóôåð ç àäðåñîì ES:DI
load_sector:
	push dx
	add ax, word[fs_base] ;  çàïèñàòè â  ÀÕ çíà÷åííÿ çì³ííî¿ fs_base - 2 áàéòè 
	adc dx, word[fs_base + 2] ; äîäàòè äî DX [fs_base + 2]
	cmp byte[sector_per_track], 0xFF
	je .use_EDD
	push bx cx si
	div [sector_per_track]
	mov cl, dl
	inc cl
	div [head_count]
	mov dh, ah
	mov ch, al
	mov dl, [disk_id]
	mov bx, di
	mov al, 1
	mov si, 3 ; 3 ñïðîáè ïðî÷èòàòè ç äèñêó
 @@:
	mov ah, 2 ; ÷èòàòè ñåêòîð
	int 0x13 ; äèñêîâèé ââ³ä/âèâ³ä
		; âõîä: DL = íîìåð äèñêà (0=äèñê A...; 80H=òâ.äèñê 0; 81H=òâ.äèñê 1)
		;            DH = íîìåð ãîëîâêè ÷òåíèÿ/çàïèñè
		;            CH = íîìåð äîðîæêè (öèëèíäðà)(0-n) =¬
		;            CL = íîìåð ñåêòîðà (1-n) ===========¦== Ñì. çàìå÷àíèå íèæå.
		;            AL = ÷èñëî ñåêòîðîâ (â ñóììå íå áîëüøå ÷åì îäèí öèëèíäð)
		;            ES:BX => àäðåñ áóôåðà âûçûâàþùåé ïðîãðàììû
		;            0:0078 => òàáëèöà ïàðàìåòðîâ äèñêåòû (äëÿ ãèáêèõ äèñêîâ)
		;            0:0104 => òàáëèöà ïàðàìåòðîâ òâ.äèñêà (äëÿ òâåðäûõ äèñêîâ)
		;    âûõîä: Carry-ôëàã=1 ïðè îøèáêå è êîä îøèáêè äèñêà â AH.
		;           ES:BX áóôåð ñîäåðæèò äàííûå, ïðî÷èòàííûå ñ äèñêà
		;           çàìå÷àíèå: íà ñåêòîð è öèëèíäð îòâîäèòñÿ ñîîòâåòñòâåííî 6 è 10 áèò:
		;                  1 1 1 1 1 1
		;                 +5-4-3-2-1-0-9-8-7-6-5-4-3-2-1-0+
		;             CX: ¦c c c c c c c c C c S s s s s s¦
		;                 +-+-+-+-+-+-+-+-¦-+-+-+-+-+-+-+-+
		;                                 +======> èñï. êàê ñòàðøèå áèòû íîìåðà öèëèíäðà 
		
	jnc @f
	xor ah, ah ;
		; 00H Ñáðîñ óñòðîéñòâà. âûçûâàåò ðåêàëèáðàöèþ êîíòðîëëåðà.
		; åñëè DL ðàâåí 80H èëè 81H, âûïîëíåí ñáðîñ êîíòð òâåðä äèñêà, èíà÷å FDC.
	int 0x13 ; äèñêîâèé ââ³ä/âèâ³ä
	dec si
	jnz @b ; òåïåð çàëèøèëîñÿ íà 1 ñïðîáó ìåíøå
 .error:
	call error ; âèâîäèìî ïîìèëêó ç òåêñòî ç íàñòóïíîãî ðÿäêà
	db "DISK ERROR",13,10,0
 @@:
	pop si cx bx dx
	ret
 .use_EDD:
	push si
	
	; ñïåö³àëüíà ñòðóêòóðà - ïàêåò äèñêîâîãî ïðîñòîðó
	mov byte[0x600], 0x10 ; ðîçì³ð ñòóêòóðè - ïîâèíåí áóòè íå ìåíøå 0õ10
	mov byte[0x601], 0 ; íå âèêîðèñòîâóºòüñÿ
	mov word[0x602], 1 ; ê³ëüê³ñòü ñåêòîð³â äëÿ ÷èòàííÿ
	mov [0x604], di ; àäðåñ áóôåðó äëÿ ÷èòåííÿ - DS:SI
	push es ; ES âêàçóº íà ñåãìåíò äàíèõ - òîìó ìîæå çàì³íèòè DS
	pop word[0x606]
	mov [0x608], ax ; 64 á³òíèé íîìåð ñåêòîðó - âñüîãî 8 áàéò = ïåðøèõ 2 áàéòè
	mov [0x60A], dx ; ùå 2 áàéòè
	mov word[0x60C], 0 ; ³ ùå 4 áàéòè
	mov word[0x60E], 0 ;
		
	mov ah, 0x42 ; ïðî÷èòàòè ñåêòîð
	mov dl, [disk_id] ; òóò âêàçóºòüñÿ íîìåð äèñêà ç ÿêîãî â³äáóäåòüñÿ ÷èòàííÿ
	mov si, 0x600 ; àäðåñ ñòðóêòóðè - ïàêåò äèñêîâîãî ïðîñòîðó
	int 0x13 ; äèñêîâèé ââ³ä/âèâ³ä
	
	jc .error
	pop si dx
	ret
	
; Ïîøóê ôàéëó ç ³ìåíåì DS:SI â êàòàëîãå DX:AX
find_file:
	push cx dx di
 .find:
	cmp ax, -1
	jne @f
	cmp dx, -1
	jne @f
 .not_found:
	call error
	db "NOT FOUND",13,10,0
 @@:
	mov di, f_info
	call load_sector
	push di
	mov cx, 0xFFFF
	xor al, al
	repne scasb
	neg cx
	dec cx
	pop di
	push si
	repe cmpsb
	pop si
	je .found
	mov ax, word[f_next]
	mov dx, word[f_next + 2]
	jmp .find
 .found:
	pop di dx cx
	ret
	
; Çàâàíòàæåííÿ ïîòî÷íîãî (çíàéäåíîãî) ôàéëó ïî àäðåñó BX:0. Â ÀÕ ïîâåðòàºòüñÿ ê³ëüê³ñòü çàâàíòàæåíèõ ñåêòîð³â
load_file_data:
	push bx cx dx si di
	mov ax, word[f_data]
	mov dx, word[f_data + 2]
 .load_list:
	cmp ax, -1
	jne @f
	cmp dx, -1
	jne @f
 .file_end:
	pop di si dx cx
	mov ax, bx
	pop bx
	sub ax, bx
	shr ax, 9 - 4
	ret
 @@:
	mov di, 0x8000 / 16
	call load_sector
	mov si, di
	mov cx, 512 / 8 - 1
 .load_sector:
	lodsw
	mov dx, [si]
	add si, 6
	cmp ax, -1
	jne @f
	cmp dx, -1
	je .file_end	
 @@:
	push es
	mov es, bx
	xor di, di
	call load_sector
	add bx, 0x200 / 16
	pop es
	loop .load_sector
	lodsw
	mov dx, [si]
	jmp .load_list
	
; òî÷êà âõîäó â ïî÷àòêîâèé çàâàíòàæóâà÷
boot:
	; íàëàøòóâàííÿ ñåãìåíòíèõ ðåã³ñòð³â
	jmp 0:@f
 @@:
	mov ax, cs
	mov ds, ax ; ds ³ es âêàçóþòü òóäè êóäè ³ cs
	mov es, ax
	
	; íàëàøòóâàííÿ ñòåêó
	mov ss, ax ; ñòåêîâèé ñåãìåíò - òîé ùî ³ ñåãìåíò ç êîäîì
	mov sp, $$ ; "äíî" ñòåêó âêàçóº íà çíà÷åííÿ çàäàíå äèðåêòèâîþ org. Ñòåê ðîñòå â íàïðÿì³ çìåíøåííÿ àäðåñ³â
	; òîìó org 0x7C00 - çàäàº 0x7C00 - ìàêñèìàëüíèé ðîçì³ð ñòåêó
	
	; äîçâîëèòè àïàðàòí³ ïåðåðèâàííÿ
	sti
	; çàïàìÿòàºìî íîìåð çàâàíòàæóâàëüíîãî äèñêó
	mov [disk_id], dl
	
	mov ah, 0x41 ; âèçíà÷èòè ïàðàìåòðè çàâàíòàæóâàëüíîãî äèñêó
	mov bx, 0x55AA
	int 0x13 ; äèñêîâå ââåäåííÿ/âèâäåííÿ
	jc @f ; ÿêùî òàêèé ñåðâ³ñ íå äîñòóïíèé ïåðåõîäèìî äî çâè÷àéíîãî
	mov byte[sector_per_track], 0xFF ; ÿêùî äîñòóïíèé
	jmp .disk_detected ; ïåðåõîäèìî äàë³
 @@:
	mov ah, 0x08 ; íîìåð ôóíêö³¿ äëÿ âèçíà÷åííÿ ïàðàìåòð³â äèñêó
		;  âõîä: DL = äèñê
		; âûõîä: DL = ÷èñëî òâ. äèñêîâ íà ïåðâîì êîíòðîëëåðå
		;        DH = ìàêñèìàëüíûé íîìåð ãîëîâêè
		;        CH = ìàêñèìàëüíûé íîìåð öèëèíäðà (ìëàäøèå 8 áèò)
		;        CL = ìàêñèì. íîìåð ñåêòîðà (è ñòàðøèå áèòû ìàêñ. íîìåðà öèëèíäðà)
	xor di, di
	push es
	int 0x13  ; äèñêîâå ââåäåííÿ/âèâäåííÿ
	pop es
	jc load_sector.error
	inc dh
	mov [head_count], dh
	and cx, 111111b
	mov [sector_per_track], cx
 .disk_detected:
	; çàâàíòàæèìî ïðîäîâæåííÿ ïî÷àòêîâîãî çàâàíòàæóâà÷à
	; ïîøóê ôàéëó boot.bin
	mov si, boot_file_name
	mov ax, word[fs_first_file]
	mov dx, word[fs_first_file + 2]
	call find_file
	mov bx, 0x7E00 / 16
	call load_file_data ; çàâàíòàæåííÿ ôàéëó boot.bin
	
	; ïåïðåõîäèìî íà çàâàíòàæóâà÷ äðóãîãî ð³âíÿ
	jmp boot2
; ïóñòèé ïðîñò³ð 
rb 510 - ($ - $$)
db 0x55,0xAA ; ñèãíàòóðà - ÿêùî ¿¿ íå áóäå BIOS ìîæå ââàæàòè çàâàíòàæóâà÷ íå êîðåêòíèì

; äîäàòêîâ³ äàí³ çàâàíòàæóâà÷à
load_msg_preffix db "Loading '",0
load_msg_suffix db "'...",0
ok_msg db "OK",13,10,0
config_file_name db "boot.cfg",0
start16_msg db "Starting 16 bit kernel...",13,10,0
start32_msg db "Starting 32 bit kernel...",13,10,0
label module_list at 0x6000
label memory_map at 0x7000

; ðîçáèòòÿ ðÿäêà DS:SI ïî ñèìâîëó ñëåø
split_file_name:
	push si
 @@:
	lodsb
	cmp al, "/"
	je @f
	test al, al
	jz @f
	jmp @b
 @@:
	mov byte[si - 1], 0
	mov ax, si
	pop si
	ret
; çàâàíòàæåííÿ ôàéëó ç ³ìÿì DS:SI â áóôåð BX:0. Ðîçì³ð ôàéëó â ñåòîðàõ ïîâåðòàºòüñÿ â AX
load_file:
	push si
	mov si, load_msg_preffix
	call write_str
	pop si
	call write_str
	push si
	mov si, load_msg_suffix
	call write_str
	pop si
	push si bp
	mov dx, word[fs_first_file + 2]
	mov ax, word[fs_first_file]
 @@:
	push ax
	call split_file_name
	mov bp, ax
	pop ax
	call find_file
	test byte[f_flags], 1
	jz @f
	mov si, bp
	mov dx, word[f_data + 2]
	mov ax, word[f_data]
	jmp @b	
 @@:
	call load_file_data
	mov si, ok_msg
	call write_str
	pop bp si
	ret
	
; îòðèìàííÿ êàðòè ïàìÿò³
get_memory_map:
	mov di, memory_map
	xor ebx, ebx ; 0 - çì³ùåííÿ â³ä ïî÷àòêó êàðòè ïàìÿò³
 @@:
	mov eax, 0xE820 ; âèçíà÷èòè êàðòó ïàìÿò³
	mov edx, 0x534D4150 ; "SMAP"
	mov ecx, 24 ; ðîçì³ð áóôåðó
	mov dword[di + 20], 1
	int 0x15 ; ðîùèðåíèé ñåðâ³ñ ÀÒ
	jc @f
	add di, 24
	test ebx, ebx
	jnz @b
 @@:
	cmp di, 0x7000
	ja .ok
	mov dword[di], 0x100000
	mov dword[di + 4], 0
	mov dword[di + 12], 0
	mov dword[di + 16], 1
	mov dword[di + 20], 0
	mov ax, 0xE801 ; âèçíà÷èòè ðîçì³ð ïàìÿò³ äî 15 Ìá
	int 0x15 ; ðîùèðåíèé ñåðâ³ñ ÀÒ
	jnc @f
	mov ah, 0x88 ; ïîâåðíóòè ðîçì³ð ðîçøèðåíî¿ ïàìÿò³
	int 0x15 ; ðîùèðåíèé ñåðâ³ñ ÀÒ
	jc .ok
	mov cx, ax
	xor dx, dx
 @@:
	test cx, cx
	jz @f
	mov ax, cx
	mov bx, dx
 @@:
	movzx eax, ax
	movzx ebx, bx
	mov ecx, 1024
	mul ecx
	push eax
	mov eax, ebx
	mov ecx, 65536
	mul ecx
	pop edx
	add eax, edx
	mov [di + 8], eax
	add di, 24
	jmp .ok
 .ok:
	xor ax, ax
	mov cx, 24 / 2
	rep stosw
	ret
	
; çàâàíòàæóâà÷ äðóãîãî åòàïó
boot2:
	; çàâàíòàæóºìî êîíô³ã ôàéë boot.cfg
	mov si, config_file_name
	mov bx, 0x1000 / 16
	call load_file
	; âèêîíàòè çàâàíòàæóâàëüíèé ñêðèïò
	mov bx, 0x9000 / 16
	mov bp, module_list
	mov dx, 0x1000
 .parse_line:
	mov si, dx
 .parse_char:
	lodsb
	test al, al
	jz .config_end
	cmp al, 10
	je .run_command
	cmp al, 13
	je .run_command
	jmp .parse_char
 .run_command:
	mov byte[si - 1], 0
	xchg dx, si
	cmp byte[si], 0
	je .parse_line ; ïóñòèé ðÿäîê
	cmp byte[si], "#"
	je .parse_line ; êîìåíòàð
	cmp byte[si], "L"
	je .load_file ; çàâàíòàæåííÿ ôàéëó
	cmp byte[si], "S"
	je .start ; Çàïóñê ÿäðà
	; Íåçíàéîìà êîìàíäà
	mov al, [si]
	mov [.cmd], al
	call error
	db "Unknown boot script command '"
	.cmd db ?
	db "'!",13,10,0
 .config_end: ; ÿêùî âñå ïðàâèëüíî ñþäùè íå ìîæíà ïîïàñòè
	; çàâåðøåííÿ
	jmp reboot
	
; çàâàíòàæåííÿ ôàéëó
 .load_file:
	push dx
	inc si
	call load_file
	push ax
	mov cx, 512
	mul cx
	mov word[bp + 8], ax
	mov word[bp + 10], dx
	mov word[bp + 12], 0
	mov word[bp + 14], 0
	mov ax, bx
	mov cx, 16
	mul cx
	mov word[bp], ax
	mov word[bp + 2], dx
	mov word[bp + 4], 0
	mov word[bp + 6], 0
	pop ax
	shr ax, 9 - 4
	add bx, ax
	add bp, 16
	pop dx
	jmp .parse_line
	
; çàïóñê ÿäðà
 .start:
	; ïåðåâ³ðèìî ÷è çàâàíòàæåíèé õî÷àá îäèí ôàéë
	cmp bx, 0x9000 / 16
	ja @f
	call error
	db "NO KERNEL LOADED",13,10,0	
 @@:
	; çàïîâíþºìî îñòàíí³é åëåìåíò ñïèñêó ôàéë³â
	xor ax, ax
	mov cx, 16
	mov di, bp
	rep stosw
	; ïåðåõ³ä äî ïðîöåäóðè ³í³ö³àë³çàö³ÿ ÿäðà ïîòð³áíî¿ ðîçðÿäíîñò³
	inc si
	cmp word[si], "16"
	je .start16
	cmp word[si], "32"
	je .start32
	;cmp word[si], "64"
	;je .start64
	; íåâ³ðíî âêàçàíà ðîçðÿäí³ñòü
	call error
	db "Invalid start command argument",13,10,0
	
; Çàïóñê 16-ðîçðÿäíîãî ÿäðà
 .start16:
	mov si, start16_msg
	mov bx, module_list
	mov dl, [disk_id]
	jmp 0x9000
	
; Çàïóñê 32-ðîçðÿäíîãî ÿäðà
 .start32:
	; Âèâîäèìî ïîâ³äîìëåííÿ ïðî çàïóñê 32-á³òíîãî ÿäðà
	mov si, start32_msg
	call write_str
	; ïåðåâ³ðÿºìî ÷è ïðîöåñîð íå ñòàð³øèé çà i386
	mov ax, 0x7202
	push ax
	popf
	pushf
	pop bx
	cmp ax, bx
	je @f
	call error
	db "Required i386 or better",13,10,0	
 @@:
	; îòðèìàòè êàðòó ïàìÿò³
	call get_memory_map
	; î÷èñòêà òàáëèö³ ñòîð³íîê
	xor ax, ax
	mov cx, 3 * 4096 / 2
	mov di, 0x1000
	rep stosw
	; çàïîâíþºìî êàòàëîã ñòîð³íîê
	mov word[0x1000], 0x2000 + 111b
	mov word[0x1FFC], 0x3000 + 111b
	; çàïîâíþºìî ïåðøó ñòîð³íêó
	mov eax, 11b
	mov cx, 0x100000 / 4096
	mov di, 0x2000
 @@:
	stosd
	add eax, 0x1000
	loop @b
	; Çàïîëâíèì îñòàííþ ñòîð³íêó
	mov di, 0x3000
	mov eax, dword[module_list]
	or eax, 11b
	mov ecx, dword[module_list + 8]
	shr ecx, 12
 @@:
	stosd
	add eax, 0x1000
	loop @b
	mov word[0x3FF4], 0x4000 + 11b ; Kernel stack
	mov word[0x3FF8], 0x3000 + 11b ; Kernel page table
	; çàâàíòàæèìî çíà÷åííÿ â CR3
	mov eax, 0x1000
	mov cr3, eax
	; çàâàíòàæèìî çíà÷åííÿ â GDTR
	lgdt [gdtr32]
	; Çàïðåòèì ïðåðûâàíèÿ
	cli
	; ïåðåéäåìî â çàõèùåíèé ðåæèì
	mov eax, cr0
	or eax, 0x80000001
	mov cr0, eax
	; ïåðåéäåìî íà 32-á³òíûé êîä
	jmp 8:start32
; òàáëèöÿ äåñêðèïòîð³â ñåãìåíò³â ÿêà ðîçì³ùóºòüñÿ òåïåð â 32 á³òíîìó ÿäð³
align 16
gdt32: ; Ì³òêà - ïî÷àòîê òàáëèö³ äåñêðèïòîð³â
	dq 0                  ; NULL - 0  - íóëüîâèé äåñêðèïòîð - çàâæäè ìàº áóòè 0
	dq 0x00CF9A000000FFFF ; CODE - 8 - äåñêðèïòîð ñåãìåíòó êîäó - 0x00000000 - 0xFFFFFFFF
	dq 0x00CF92000000FFFF ; DATA - 16 - äåñêðèïòîð íà ñåãìåíòó äàíèõ - 0x00000000 - 0xFFFFFFFF
gdtr32: ; ñòðóêòóðà ÿêà çàâàíòàæóþòüñÿ ïðè lgdt - load global descritor table
	dw $ - gdt32 - 1 ; ðîçì³ð òàáëèö³ - â ïàìÿò³ çàéìàº 2 áàéòè
	dd gdt32 ; àäðåñ òàáëèö³ äåñêðèïòîð³â - â ïàìÿò³ çàéìàº 4 áàéòè
; 32-á³òíèé êîä
use32
start32:
	; íàëàøòîâóºìî ñåãìåíòí³ ðåã³ñòè òà ñòåê
	mov eax, 16 ; çì³ùåííÿ äî äåñêðèïòîðà äàíèõ â òàáèö³ äåñêðèïòîð³â - gdt32
	mov ds, ax ; âñ³ ñåãìåíòí³ ðåã³ñòðè îêð³ì CS âêàçóþòü íà îäèí ñåãìåíò äàíèõ
	mov es, ax
	mov fs, ax
	mov gs, ax
	mov ss, ax
	mov esp, 0xFFFFDFFC ; àäðåñà ñòåêó
	; Çàïèñ ïàðàìåòð³â äëÿ ôóíêö³¿ kernel_main
	; çàïèñóºìî â DL íîìåð çàâàíòàæóâàëüíîãî äèñêó
	mov dl, [disk_id]
	; â EBX àäðåñ ñïèñêó çàâàíòàæåíèõ ôàéëîâ
	mov ebx, module_list
	; â ESI àäðåñ êàðòè ïàìÿò³
	mov esi, memory_map
	; Ïåðåõîäèìî íà ÿäðî - íà êîä ³ç  startup.asm
	jmp 0xFFC00000
