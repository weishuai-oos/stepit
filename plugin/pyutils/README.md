# pyutils

StepIt plugin containing Python-based utilities for C++ modules.

### Provided Factories

- `stepit::field::DataLoader`:
  - `npz`: loads keyed arrays from an `.npz` file.

### Executables

- `npz_info`: prints key/shape/dtype summary for arrays in an `.npz` file.

	```shell
	npz_info <npz_file>
	```
