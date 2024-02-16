.arch armv6
.text
.align 2
.global countMatches
.type countMatches, %function
countMatches:
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 1, uses_anonymous_args = 0

	@ Save the previous frame pointer and link register
	push {fp, lr}

	@ Set up the new frame pointer
	add fp, sp, #4

	@ Allocate space for local variables
	sub sp, sp, #40

	@ Save the first argument (array1) in stack frame
	str r0, [fp, #-40]

	@ Save the second argument (array2) in stack frame
	str r1, [fp, #-44]

	@ Initialize the outer loop index variable to 3
	mov r3, #3
	str r3, [fp, #-28]

	@ Initialize the outer loop counter to 0
	mov r3, #0
	str r3, [fp, #-8]

	@ Initialize the inner loop index variable to 0
	mov r3, #0
	str r3, [fp, #-12]

	@ Allocate memory for a temporary array
	ldr r3, [fp, #-28]
	lsl r3, r3, #2
	mov r0, r3
	bl malloc
	mov r3, r0
	str r3, [fp, #-32]

	@ Outer loop start
	b .L2
.L2:
	ldr	r2, [fp, #-16]
	ldr	r3, [fp, #-28]
	cmp	r2, r3
	blt	.L3
	mov	r3, #0
	str	r3, [fp, #-20]
	b	.L4

.L3:
	@ Load the outer loop index
	ldr r3, [fp, #-16]

	@ Calculate the memory address for the temporary array
	lsl r3, r3, #2
	ldr r2, [fp, #-32]
	add r3, r2, r3

	@ Set the value at the memory address to 0
	mov r2, #0
	str r2, [r3]

	@ Increment the outer loop index
	ldr r3, [fp, #-16]
	add r3, r3, #1
	str r3, [fp, #-16]

	@ Check if the outer loop index is less than the outer loop counter
	ldr r2, [fp, #-16]
	ldr r3, [fp, #-28]
	cmp r2, r3
	blt .L3

	@ Initialize the match count to 0
	mov r3, #0
	str r3, [fp, #-20]

	@ Outer loop condition check
	b .L4

.L11:
	@ Load the outer loop index
	ldr r3, [fp, #-20]

	@ Calculate the memory address for array1
	lsl r3, r3, #2
	ldr r2, [fp, #-40]
	add r3, r2, r3

	@ Load the value from array1
	ldr r2, [r3]

	@ Load the outer loop index
	ldr r3, [fp, #-20]

	@ Calculate the memory address for array2
	lsl r3, r3, #2
	ldr r1, [fp, #-44]
	add r3, r1, r3

	@ Load the value from array2
	ldr r3, [r3]

	@ Compare the values from array1 and array2
	cmp r2, r3

	@ Branch to .L5 if they are not equal
	bne .L5

	@ Increment the match count
	ldr r3, [fp, #-8]
	add r3, r3, #1
	str r3, [fp, #-8]

	@ Load the outer loop index
	ldr r3, [fp, #-20]

	@ Calculate the memory address for the temporary array
	lsl r3, r3, #2
	ldr r2, [fp, #-32]
	add r3, r2, r3

	@ Load the value from the temporary array
	ldr r3, [r3]

	@ Check if the value is not 0
	cmp r3, #0
	beq .L6

	@ Decrement the inner loop counter
	ldr r3, [fp, #-12]
	sub r3, r3, #1
	str r3, [fp, #-12]

.L6:
	@ Load the outer loop index
	ldr r3, [fp, #-20]

	@ Calculate the memory address for the temporary array
	lsl r3, r3, #2
	ldr r2, [fp, #-32]
	add r3, r2, r3

	@ Set the value in the temporary array to 1
	mov r2, #1
	str r2, [r3]

	@ Branch to .L7
	b .L7

.L5:
	@ Initialize the temporary index to 0
	mov r3, #0
	str r3, [fp, #-24]

	@ Inner loop condition check
	b .L8

.L10:
	@ Load the inner loop index
	ldr r3, [fp, #-24]

	@ Calculate the memory address for array2
	lsl r3, r3, #2
	ldr r2, [fp, #-44]
	add r3, r2, r3

	@ Load the value from array2
	ldr r2, [r3]

	@ Load the inner loop index
	ldr r3, [fp, #-24]

	@ Calculate the memory address for array1
	lsl r3, r3, #2
	ldr r1, [fp, #-40]
	add r3, r1, r3

	@ Load the value from array1
	ldr r3, [r3]

	@ Compare the values from array2 and array1
	cmp r2, r3

	@ Branch to .L9 if they are not equal
	bne .L9

	@ Load the inner loop index
	ldr r3, [fp, #-24]

	@ Calculate the memory address for the temporary array
	lsl r3, r3, #2
	ldr r2, [fp, #-32]
	add r3, r2, r3

	@ Load the value from the temporary array
	ldr r3, [r3]

	@ Check if the value is 0
	cmp r3, #0
	bne .L9

	@ Load the inner loop index
	ldr r3, [fp, #-24]

	@ Calculate the memory address for the temporary array
	lsl r3, r3, #2
	ldr r2, [fp, #-32]
	add r3, r2, r3

	@ Set the value in the temporary array to 1
	mov r2, #1
	str r2, [r3]

	@ Increment the match count
	ldr r3, [fp, #-12]
	add r3, r3, #1
	str r3, [fp, #-12]

	@ Branch to .L7
	b .L7

.L9:
	@ Increment the inner loop index
	ldr r3, [fp, #-24]
	add r3, r3, #1
	str r3, [fp, #-24]

.L8:
	@ Load the inner loop index
	ldr r2, [fp, #-24]

	@ Load the outer loop counter
	ldr r3, [fp, #-28]

	@ Compare the inner loop index with the outer loop counter
	cmp r2, r3

	@ Branch to .L10 if the inner loop index is less than the outer loop counter
	blt .L10

.L7:
	@ Increment the outer loop index
	ldr r3, [fp, #-20]
	add r3, r3, #1
	str r3, [fp, #-20]

.L4:
	@ Load the outer loop index
	ldr r2, [fp, #-20]

	@ Load the outer loop counter
	ldr r3, [fp, #-28]

	@ Compare the outer loop index with the outer loop counter
	cmp r2, r3

	@ Branch to .L11 if the outer loop index is less than the outer loop counter
	blt .L11

	@ Allocate memory for the final result
	mov r0, #8
	bl malloc
	mov r3, r0
	str r3, [fp, #-36]

	@ Store the match count in the first element of the result array
	ldr r3, [fp, #-36]
	ldr r2, [fp, #-8]
	str r2, [r3]

	@ Calculate the memory address for the second element of the result array
	ldr r3, [fp, #-36]
	add r3, r3, #4

	@ Store the inner loop counter in the second element of the result array
	ldr r2, [fp, #-12]
	str r2, [r3]

	@ Free the memory allocated for the temporary array
	ldr r0, [fp, #-32]
	bl free

	@ Load the result array address into r0
	ldr r3, [fp, #-36]
	mov r0, r3

	@ Restore the stack pointer
	sub sp, fp, #4

	@ Restore the previous frame pointer and return
	pop {fp, pc}

	.size countMatches, .-countMatches
